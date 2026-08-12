// HH_260810 - Added read-only Lucid device-to-interface inventory without opening or reconfiguring a camera.
#include <ArenaApi.h>
#include <GenApi/GenApi.h>

#include <cstdint>
#include <iostream>
#include <string>

namespace
{
std::string ipv4(const std::uint32_t value)
{
  return std::to_string((value >> 24) & 0xffU) + "." +
         std::to_string((value >> 16) & 0xffU) + "." +
         std::to_string((value >> 8) & 0xffU) + "." +
         std::to_string(value & 0xffU);
}

std::string integer_node(GenApi::INodeMap * const node_map, const char * const name)
{
  GenApi::CIntegerPtr node = node_map->GetNode(name);
  if (!node || !GenApi::IsReadable(node)) {
    return "unavailable";
  }
  return ipv4(static_cast<std::uint32_t>(node->GetValue()));
}
}  // namespace

int main()
{
  Arena::ISystem * system = nullptr;
  try {
    system = Arena::OpenSystem();
    system->UpdateDevices(250);
    auto devices = system->GetDevices();
    std::cout << "device_count=" << devices.size() << '\n';

    for (auto & device : devices) {
      GenApi::INodeMap * const interface_map = system->GetTLInterfaceNodeMap(device);
      std::cout << "serial=" << device.SerialNumber()
                << " model=" << device.ModelName()
                << " mac=" << device.MacAddressStr()
                << " device_ip=" << device.IpAddressStr()
                << " device_mask=" << device.SubnetMaskStr();
      if (interface_map) {
        std::cout << " interface_ip="
                  << integer_node(interface_map, "GevInterfaceSubnetIPAddress")
                  << " interface_mask="
                  << integer_node(interface_map, "GevInterfaceSubnetMask");
      } else {
        std::cout << " interface_ip=unavailable interface_mask=unavailable";
      }
      std::cout << '\n';
    }

    Arena::CloseSystem(system);
    return 0;
  } catch (const GenICam::GenericException & error) {
    std::cerr << "Arena enumeration failed: " << error.what() << '\n';
  } catch (const std::exception & error) {
    std::cerr << "Enumeration failed: " << error.what() << '\n';
  }

  if (system) {
    Arena::CloseSystem(system);
  }
  return 1;
}
