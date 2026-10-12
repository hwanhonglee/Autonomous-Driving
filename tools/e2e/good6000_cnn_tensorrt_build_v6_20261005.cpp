// HH_261005 - Fresh V6 uniquely names hh_json_quote to avoid std::quoted ADL; all original native math/guards unchanged. Fixed6/30 FP32 parser/build feasibility only using the explicit existing vendor10.8 SDK; no execution context, inference enqueue or controller.
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr char lock_path[] = "/home/a/autoware_e2e/storage/assets/locks/autoware_e2e_runtime.lock";
void need(bool value, char const* text) { if (!value) throw std::runtime_error(text); }
std::string hh_json_quote(std::string const& value) {
  std::ostringstream out; out << '"';
  for (unsigned char ch : value) { if (ch == '"' || ch == '\\') out << '\\' << ch;
    else if (ch < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(ch); else out << ch; }
  out << '"'; return out.str();
}
class Logger final : public nvinfer1::ILogger {
  void log(Severity level, char const* text) noexcept override { if (level <= Severity::kWARNING) std::cerr << text << '\n'; }
};
void owner_guard(char** argv) {
  auto pid = std::stoll(argv[5]); need(pid == getppid(), "exact owned parent PID required");
  std::ifstream input("/proc/" + std::to_string(pid) + "/stat"); std::string text; std::getline(input, text);
  need(input.good() || input.eof(), "parent generation read failed");
  auto last = text.rfind(')'); need(last != std::string::npos, "parent stat malformed");
  std::istringstream fields(text.substr(last + 2)); std::string item;
  for (int index = 0; index <= 19; ++index) need(bool(fields >> item), "parent start_ticks missing");
  need(std::stoull(item) == std::stoull(argv[6]), "owned parent generation changed");
  int inherited = std::stoi(argv[7]); struct stat held{}, target{};
  need(fstat(inherited, &held) == 0 && stat(lock_path, &target) == 0 && held.st_dev == target.st_dev && held.st_ino == target.st_ino,
       "exact inherited runtime lock FD required");
  int check = open(lock_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW); need(check >= 0, "readonly lock check open failed");
  bool conflict = flock(check, LOCK_EX | LOCK_NB) < 0 && (errno == EWOULDBLOCK || errno == EAGAIN); close(check);
  need(conflict, "exclusive shared runtime lease required");
  need(std::getenv("CUDA_VISIBLE_DEVICES") && std::string(std::getenv("CUDA_VISIBLE_DEVICES")) == argv[1], "exact GPU UUID mask required");
}
void write_new(std::string const& path, void const* data, std::size_t size) {
  need(size && size <= 32U * 1024U * 1024U, "bounded standalone engine required");
  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600); need(fd >= 0, "fresh engine output required");
  auto bytes = static_cast<char const*>(data); std::size_t done = 0;
  while (done < size) { auto count = write(fd, bytes + done, size - done); if (count <= 0) { close(fd); throw std::runtime_error("engine write failed"); } done += count; }
  bool flushed = fsync(fd) == 0; bool closed = close(fd) == 0; need(flushed && closed, "engine durable close failed");
}
bool shape(nvinfer1::Dims const& value, std::initializer_list<int64_t> expected) {
  if (value.nbDims != int(expected.size())) return false; int index = 0;
  for (auto dimension : expected) if (value.d[index++] != dimension) return false; return true;
}
void build(int batch, std::string const& input_path, std::string const& output_path, Logger& logger, bool& precision_verified) {
  struct stat size{}; need(lstat(input_path.c_str(), &size) == 0 && S_ISREG(size.st_mode)
    && size.st_size > 0 && size.st_size <= 32U * 1024U * 1024U, "bounded regular ONNX required before read");
  std::ifstream input(input_path, std::ios::binary); need(bool(input), "sealed ONNX read failed");
  std::vector<char> data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  need(!data.empty() && data.size() <= 32U * 1024U * 1024U, "bounded sealed ONNX required");
  std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger)); need(bool(builder), "builder creation failed");
  std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(1U << unsigned(nvinfer1::NetworkDefinitionCreationFlag::kSTRONGLY_TYPED)));
  need(bool(network), "strongly typed network required");
  std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, logger)); need(bool(parser), "parser creation failed");
  parser->setFlag(nvonnxparser::OnnxParserFlag::kNATIVE_INSTANCENORM);
  need(parser->getFlag(nvonnxparser::OnnxParserFlag::kNATIVE_INSTANCENORM), "native InstanceNorm required; no custom plugin substitution");
  bool parsed = parser->parse(data.data(), data.size(), nullptr);
  for (int index = 0; index < parser->getNbErrors(); ++index) std::cerr << "parser_error[" << index << "]: " << parser->getError(index)->desc() << '\n';
  need(parsed && parser->getNbErrors() == 0, "ONNX parse failed; all errors retained");
  need(network->getNbInputs() == 1 && network->getNbOutputs() == 1, "one standalone CNN input/output required");
  auto in = network->getInput(0); auto out = network->getOutput(0);
  need(std::string(in->getName()) == "normalized_images" && std::string(out->getName()) == "cnn_features"
       && in->getType() == nvinfer1::DataType::kFLOAT && out->getType() == nvinfer1::DataType::kFLOAT
       && shape(in->getDimensions(), {batch, 3, 180, 320}) && shape(out->getDimensions(), {batch, 60, 128}), "exact fixed FP32 CNN ABI required");
  for (int layer = 0; layer < network->getNbLayers(); ++layer) for (int tensor = 0; tensor < network->getLayer(layer)->getNbOutputs(); ++tensor) {
    auto type = network->getLayer(layer)->getOutput(tensor)->getType();
    need(type == nvinfer1::DataType::kFLOAT || type == nvinfer1::DataType::kINT32 || type == nvinfer1::DataType::kINT64 || type == nvinfer1::DataType::kBOOL,
         "lower precision floating or quantized tensor denied"); }
  std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig()); need(bool(config), "builder config failed");
  for (auto flag : {nvinfer1::BuilderFlag::kTF32, nvinfer1::BuilderFlag::kFP16, nvinfer1::BuilderFlag::kINT8,
                   nvinfer1::BuilderFlag::kBF16, nvinfer1::BuilderFlag::kFP8}) { config->clearFlag(flag); need(!config->getFlag(flag), "FP32 precision flag readback failed"); }
  precision_verified = true;
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 64U * 1024U * 1024U);
  std::unique_ptr<nvinfer1::IHostMemory> engine(builder->buildSerializedNetwork(*network, *config)); need(bool(engine), "FP32 engine build failed");
  write_new(output_path, engine->data(), engine->size());
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 8) { std::cerr << "fixed owned parent/GPU/input/output arguments required\n"; return 2; }
  Logger logger; std::string setup_error;
  try { owner_guard(argv); need(prctl(PR_SET_PDEATHSIG, SIGKILL) == 0, "owned parent-death signal setup failed");
    owner_guard(argv); // HH_261005 - Recheck the exact generation after installing PDEATHSIG to close the parent-death race before CUDA.
    int devices = 0; need(cudaGetDeviceCount(&devices) == cudaSuccess && devices == 1, "only one UUID-masked GPU allowed");
    need(cudaSetDevice(0) == cudaSuccess, "declared visible GPU0 selection failed"); }
  catch (std::exception const& error) { setup_error = error.what(); }
  bool failed = false;
  for (int index = 0; index < 2; ++index) {
    int batch = index ? 30 : 6; std::string error; bool precision_verified = false; auto begin = std::chrono::steady_clock::now();
    try { need(setup_error.empty(), setup_error.c_str()); owner_guard(argv);
      build(batch, argv[index + 2], std::string(argv[4]) + "/cnn_fixed" + std::to_string(batch) + ".engine", logger, precision_verified); }
    catch (std::exception const& failure) { error = failure.what(); failed = true; }
    auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    std::cout << "{\"batch\":" << batch << ",\"status\":" << hh_json_quote(error.empty() ? "PARSED_BUILT_FP32_FEASIBILITY_ONLY" : "FAIL_RETAINED")
              << ",\"error\":" << (error.empty() ? "null" : hh_json_quote(error)) << ",\"seconds\":" << std::setprecision(17) << seconds
              << ",\"precision_flags_readback_verified\":" << (precision_verified ? "true" : "false")
              << ",\"TF32_FP16_INT8_BF16_FP8_enabled\":" << (precision_verified ? "false" : "null")
              << ",\"runtime_or_cached_output_parity_verified\":false}\n" << std::flush;
  }
  return failed ? 2 : 0;
}
