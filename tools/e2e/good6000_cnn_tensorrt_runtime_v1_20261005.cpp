// HH_261005 - SOURCE ONLY: persistent fixed6/30 FP32 CNN byte protocol, not a compiler/run grant. Python V1 actual entry is unconditionally denied pending a fresh ROOT-reviewed owner/engine/source/asset/lease declaration.
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <openssl/sha.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr char lock_path[] = "/home/a/autoware_e2e/storage/assets/locks/autoware_e2e_runtime.lock";
constexpr uint32_t magic = 0x4848434e, version = 1;
using Clock = std::chrono::steady_clock;
using Digest = std::array<unsigned char,32>;
void need(bool value, char const* message) { if (!value) throw std::runtime_error(message); }
class Logger final : public nvinfer1::ILogger {
  void log(Severity level, char const* text) noexcept override { if (level <= Severity::kWARNING) std::cerr << text << '\n'; }
};
Digest digest(void const* bytes, size_t count) {
  Digest result{}; need(SHA256(static_cast<unsigned char const*>(bytes),count,result.data()) != nullptr,"SHA256 failed"); return result;
}
std::string hex(Digest const& value) {
  std::ostringstream out; for (auto byte:value) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte); return out.str();
}
void owner_guard(char** argv) {
  auto pid=std::stoll(argv[6]); need(pid==getppid(),"exact owned parent required");
  std::ifstream input("/proc/"+std::to_string(pid)+"/stat"); std::string text; std::getline(input,text);
  auto last=text.rfind(')'); need(last!=std::string::npos,"parent generation unavailable");
  std::istringstream fields(text.substr(last+2)); std::string item;
  for (int index=0;index<=19;++index) need(bool(fields>>item),"parent generation malformed");
  need(std::stoull(item)==std::stoull(argv[7]),"parent generation changed");
  int held_fd=std::stoi(argv[8]); struct stat held{},target{};
  need(fstat(held_fd,&held)==0 && lstat(lock_path,&target)==0 && S_ISREG(target.st_mode)
    && held.st_dev==target.st_dev && held.st_ino==target.st_ino,"exact inherited lease FD required");
  int check=open(lock_path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW); need(check>=0,"readonly lease open failed");
  bool conflict=flock(check,LOCK_EX|LOCK_NB)<0 && (errno==EWOULDBLOCK || errno==EAGAIN); close(check);
  need(conflict,"exclusive runtime lease required");
  need(std::getenv("CUDA_VISIBLE_DEVICES") && std::string(std::getenv("CUDA_VISIBLE_DEVICES"))==argv[1],"exact physicalGPU0 UUID mask required");
  need(std::getenv("NVIDIA_TF32_OVERRIDE") && std::string(std::getenv("NVIDIA_TF32_OVERRIDE"))=="0","TF32 override must remain disabled");
}
void transfer(int fd, void* bytes, size_t count, bool writing, Clock::time_point deadline) {
  auto data=static_cast<unsigned char*>(bytes); size_t done=0;
  while (done<count) {
    auto remaining=std::chrono::duration<double,std::milli>(deadline-Clock::now()).count(); need(remaining>0,"fixed byte-protocol deadline exceeded");
    pollfd descriptor{fd,short(writing?POLLOUT:POLLIN),0};
    int result=poll(&descriptor,1,std::max(1,int(std::min(remaining,50.0))));
    if (result<0 && errno==EINTR) continue; need(result>=0,"protocol poll failed"); if (!result) continue;
    need(!(descriptor.revents&(POLLERR|POLLNVAL)),"protocol FD error");
    auto amount=writing?write(fd,data+done,count-done):read(fd,data+done,count-done);
    if (amount<0 && (errno==EINTR || errno==EAGAIN)) continue; need(amount>0,"protocol EOF/write failure"); done+=size_t(amount);
  }
}
uint32_t u32(unsigned char const* bytes) { uint32_t value=0; for(int i=0;i<4;++i) value|=uint32_t(bytes[i])<<(8*i); return value; }
uint64_t u64(unsigned char const* bytes) { uint64_t value=0; for(int i=0;i<8;++i) value|=uint64_t(bytes[i])<<(8*i); return value; }
void put(unsigned char* bytes,uint64_t value,int count) { for(int i=0;i<count;++i) bytes[i]=static_cast<unsigned char>(value>>(8*i)); }
void response(uint64_t sequence,uint32_t batch,uint32_t status,void const* payload,uint32_t size,Digest const& input,Clock::time_point deadline) {
  std::array<unsigned char,92> header{}; put(header.data(),magic,4);put(header.data()+4,version,4);put(header.data()+8,sequence,8);
  put(header.data()+16,batch,4);put(header.data()+20,size,4);put(header.data()+24,status,4);std::memcpy(header.data()+28,input.data(),32);
  auto output=digest(payload,size);std::memcpy(header.data()+60,output.data(),32);
  transfer(STDOUT_FILENO,header.data(),header.size(),true,deadline);
  if(size) transfer(STDOUT_FILENO,const_cast<void*>(payload),size,true,deadline);
}
std::vector<unsigned char> engine_bytes(char const* path,char const* expected_sha) {
  need(std::strlen(expected_sha)==64,"exact engine SHA required");
  int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);need(fd>=0,"readonly engine open failed");struct stat before{},after{};
  if(fstat(fd,&before)!=0 || !S_ISREG(before.st_mode) || before.st_size<=0 || before.st_size>32*1024*1024) {close(fd);throw std::runtime_error("bounded regular engine required");}
  std::vector<unsigned char> data(size_t(before.st_size));size_t done=0;
  while(done<data.size()) {auto count=read(fd,data.data()+done,data.size()-done);if(count<0&&errno==EINTR)continue;
    if(count<=0){close(fd);throw std::runtime_error("complete engine read failed");}done+=size_t(count);}
  bool stable=fstat(fd,&after)==0 && before.st_dev==after.st_dev && before.st_ino==after.st_ino && before.st_size==after.st_size
    && before.st_mtim.tv_sec==after.st_mtim.tv_sec && before.st_mtim.tv_nsec==after.st_mtim.tv_nsec
    && before.st_ctim.tv_sec==after.st_ctim.tv_sec && before.st_ctim.tv_nsec==after.st_ctim.tv_nsec;close(fd);
  need(stable && hex(digest(data.data(),data.size()))==expected_sha,"exact immutable engine read SHA/generation required");return data;
}
bool shape(nvinfer1::Dims const& value,std::initializer_list<int64_t> expected) {
  if(value.nbDims!=int(expected.size()))return false;int index=0;for(auto dimension:expected)if(value.d[index++]!=dimension)return false;return true;
}
struct Runner {
  int batch;std::unique_ptr<nvinfer1::ICudaEngine> engine;std::unique_ptr<nvinfer1::IExecutionContext> context;
  void* input=nullptr;void* output=nullptr;cudaStream_t stream=nullptr;
  Runner(int count,char const* path,char const* pin,nvinfer1::IRuntime& runtime):batch(count) {
    auto bytes=engine_bytes(path,pin);engine.reset(runtime.deserializeCudaEngine(bytes.data(),bytes.size()));need(bool(engine),"engine deserialize failed");
    need(engine->getNbIOTensors()==2 && engine->getNbOptimizationProfiles()==1,"exact fixed engine IO/profile count required");
    for(auto name:{"normalized_images","cnn_features"}) need(engine->getTensorDataType(name)==nvinfer1::DataType::kFLOAT
      && engine->getTensorFormat(name)==nvinfer1::TensorFormat::kLINEAR && engine->getTensorLocation(name)==nvinfer1::TensorLocation::kDEVICE,"fixed device linear FP32 IO required");
    need(engine->getTensorIOMode("normalized_images")==nvinfer1::TensorIOMode::kINPUT && engine->getTensorIOMode("cnn_features")==nvinfer1::TensorIOMode::kOUTPUT
      && shape(engine->getTensorShape("normalized_images"),{batch,3,180,320}) && shape(engine->getTensorShape("cnn_features"),{batch,60,128}),"exact original CNN names/shapes required");
    context.reset(engine->createExecutionContext());need(bool(context),"execution context creation failed");
    need(cudaMalloc(&input,size_t(batch)*3*180*320*4)==cudaSuccess && cudaMalloc(&output,size_t(batch)*60*128*4)==cudaSuccess
      && cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking)==cudaSuccess,"bounded device buffers/stream failed");
    need(context->setTensorAddress("normalized_images",input) && context->setTensorAddress("cnn_features",output),"exact engine buffer bindings required");
  }
  ~Runner(){if(stream)cudaStreamDestroy(stream);if(input)cudaFree(input);if(output)cudaFree(output);}
  std::vector<float> run(std::vector<float> const& values,Clock::time_point deadline) {
    need(values.size()==size_t(batch)*3*180*320,"exact full input float count required");for(float value:values)need(std::isfinite(value),"nonfinite normalized input denied");
    std::vector<float> result(size_t(batch)*60*128);
    need(cudaMemcpyAsync(input,values.data(),values.size()*4,cudaMemcpyHostToDevice,stream)==cudaSuccess && context->enqueueV3(stream)
      && cudaMemcpyAsync(result.data(),output,result.size()*4,cudaMemcpyDeviceToHost,stream)==cudaSuccess,"CNN transfer/enqueue failed");
    for(;;){auto state=cudaStreamQuery(stream);if(state==cudaSuccess)break;need(state==cudaErrorNotReady,"CUDA stream failed");
      need(Clock::now()<deadline,"fixed native CNN deadline exceeded");usleep(100);}
    for(float value:result)need(std::isfinite(value),"nonfinite original CNN features denied");return result;
  }
};
} // namespace

int main(int argc,char** argv) {
  // HH_261005 - This source is not an activation authority. A future ROOT-reviewed Python owner must pin both successful FP32 engine proofs, binary/SDK/assets, physicalGPU0 inventories, own generation and inherited lease before launching it.
  if(argc!=10 || std::string(argv[9])!="HH_CNN_FP32_BYTE_V1"){std::cerr<<"fixed source-owned protocol arguments required\n";return 2;}
  Logger logger;uint64_t sequence=0;uint32_t batch=0;Digest input_sha{};auto started=Clock::now();
  try {
    uint32_t endian=1;
    need(sizeof(float)==4 && std::numeric_limits<float>::is_iec559 && *reinterpret_cast<unsigned char*>(&endian)==1,"little-endian IEEE754 FP32 host required");
    int flag=fcntl(STDIN_FILENO,F_GETFL);need(flag>=0 && fcntl(STDIN_FILENO,F_SETFL,flag|O_NONBLOCK)==0,"nonblocking request pipe required");
    flag=fcntl(STDOUT_FILENO,F_GETFL);need(flag>=0 && fcntl(STDOUT_FILENO,F_SETFL,flag|O_NONBLOCK)==0,"nonblocking response pipe required");
    owner_guard(argv);need(prctl(PR_SET_PDEATHSIG,SIGKILL)==0,"owned parent death signal failed");owner_guard(argv);
    int devices=0;need(cudaGetDeviceCount(&devices)==cudaSuccess && devices==1 && cudaSetDevice(0)==cudaSuccess,"one declared physicalGPU0 only");
    std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));need(bool(runtime),"explicit vendor runtime creation failed");
    Runner warm(6,argv[2],argv[4],*runtime),cold(30,argv[3],argv[5],*runtime);owner_guard(argv);
    need(Clock::now()<started+std::chrono::seconds(120),"fixed native setup deadline exceeded");
    response(0,0,3,nullptr,0,input_sha,Clock::now()+std::chrono::seconds(1));uint64_t previous=0;
    for(;;){
      owner_guard(argv);need(Clock::now()<started+std::chrono::seconds(600),"fixed offline native lifetime exceeded");
      std::array<unsigned char,56> header{};transfer(STDIN_FILENO,header.data(),1,false,Clock::now()+std::chrono::seconds(10));
      auto deadline=Clock::now()+std::chrono::milliseconds(100);transfer(STDIN_FILENO,header.data()+1,header.size()-1,false,deadline);
      sequence=u64(header.data()+8);batch=u32(header.data()+16);auto count=u32(header.data()+20);std::memcpy(input_sha.data(),header.data()+24,32);
      need(u32(header.data())==magic && u32(header.data()+4)==version && sequence==previous+1 && (batch==6 || batch==30)
        && count==batch*3U*180U*320U*4U,"exact ordered finite FP32 request header required");
      std::vector<float> values(count/4);transfer(STDIN_FILENO,values.data(),count,false,deadline);
      need(digest(values.data(),count)==input_sha,"full original normalized input SHA differs");owner_guard(argv);
      auto result=(batch==6?warm:cold).run(values,deadline);owner_guard(argv);
      need(digest(values.data(),count)==input_sha && Clock::now()<deadline,"input mutation/fixed request deadline denied");
      response(sequence,batch,1,result.data(),uint32_t(result.size()*4),input_sha,deadline);previous=sequence;
    }
  }catch(std::exception const& error){
    std::cerr<<"FAIL_RETAINED sequence="<<sequence<<" batch="<<batch<<" "<<error.what()<<'\n';
    try{std::string message=error.what();if(message.size()>1024)message.resize(1024);
      response(sequence,batch,2,message.data(),uint32_t(message.size()),input_sha,Clock::now()+std::chrono::seconds(1));}catch(...){ }
    return 2;
  }
}
