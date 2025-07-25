// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVSystem/MemoryInfo.hpp>
#include <AVSystem/system.hpp>

#include <cmath>
#include <iomanip>

#if defined(__WINDOWS__)
    #include <windows.h>
#elif defined(__LINUX__)
    #include <sys/sysinfo.h>
    #include <fstream>
    #include <limits>
#elif defined(__APPLE__)
    #include <sys/types.h>
    #include <sys/sysctl.h>
    #include <sys/vmmeter.h>
    #include <mach/vm_statistics.h>
    #include <mach/mach_types.h>
    #include <mach/mach_init.h>
    #include <mach/mach_host.h>
#else
    #warning "System unrecognized. Can't found memory infos."
    #include <limits>
#endif

namespace aliceVision {
namespace system {

#if defined(__LINUX__)
unsigned long linuxGetAvailableRam()
{
    std::string token;
    std::ifstream file("/proc/meminfo");
    while (file >> token)
    {
        if (token == "MemAvailable:")
        {
            unsigned long mem;
            if (file >> mem)
            {
                // read in kB and convert to bytes
                return mem * 1024;
            }
            else
            {
                return 0;
            }
        }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;  // nothing found
}
#endif

MemoryInfo getMemoryInfo()
{
    MemoryInfo infos;

#if defined(__WINDOWS__)
    MEMORYSTATUS memory;
    GlobalMemoryStatus(&memory);

    // memory.dwMemoryLoad;
    infos.totalRam = memory.dwTotalPhys;
    infos.availableRam = infos.freeRam = memory.dwAvailPhys;
    // memory.dwTotalPageFile;
    // memory.dwAvailPageFile;
    infos.totalSwap = memory.dwTotalVirtual;
    infos.freeSwap = memory.dwAvailVirtual;
#elif defined(__LINUX__)
    struct sysinfo sys_info;
    sysinfo(&sys_info);

    infos.totalRam = sys_info.totalram * sys_info.mem_unit;
    infos.freeRam = sys_info.freeram * sys_info.mem_unit;

    infos.availableRam = linuxGetAvailableRam();
    if (infos.availableRam == 0)
        infos.availableRam = infos.freeRam;

    // infos.sharedRam = sys_info.sharedram * sys_info.mem_unit;
    // infos.bufferRam = sys_info.bufferram * sys_info.mem_unit;
    infos.totalSwap = sys_info.totalswap * sys_info.mem_unit;
    infos.freeSwap = sys_info.freeswap * sys_info.mem_unit;
#elif defined(__APPLE__)
    // Physical memory
    uint64_t memsize = 0;
    size_t len = sizeof(memsize);
    int mib[2] = { CTL_HW, HW_MEMSIZE };
    if (sysctl(mib, 2, &memsize, &len, NULL, 0) == 0 && len == sizeof(memsize)) {
        infos.totalRam = memsize;
    } else {
        infos.totalRam = size_t(16192) * size_t(1024) * size_t(1024); // Assume 16G
    }

    // Virtual memory.
    mib[0] = CTL_VM;
    mib[1] = VM_SWAPUSAGE;
    struct xsw_usage swap;
    len = sizeof(struct xsw_usage);
    const size_t miblen = std::size(mib);
    if (sysctl(mib, static_cast<unsigned int>(miblen), &swap, &len, NULL, 0) == 0)
    {
        infos.totalSwap = swap.xsu_total;
        infos.freeSwap = swap.xsu_avail;
    }

    // In use.
    mach_port_t host_port = mach_host_self();
    mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
    vm_statistics64_data_t vm_stat;

    if (host_statistics64(host_port, HOST_VM_INFO64, reinterpret_cast<host_info64_t>(&vm_stat), &count) != KERN_SUCCESS) {
        infos.freeRam = infos.totalRam * 0.75;
        infos.availableRam = infos.totalRam * 0.75;
    }

    vm_size_t page_size;
    if (host_page_size(host_port, &page_size) != KERN_SUCCESS) {
        infos.freeRam = infos.totalRam * 0.75;
        infos.availableRam = infos.totalRam * 0.75;
    }

    uint64_t free_pages = vm_stat.free_count;
    uint64_t inactive_pages = vm_stat.inactive_count;
    uint64_t speculative_pages = vm_stat.speculative_count;

    uint64_t available_pages = free_pages + inactive_pages + speculative_pages;
    // Some tools also add compressed memory, but this is debatable.
    const uint64_t availableMem = available_pages * static_cast<uint64_t>(page_size);
    infos.freeRam = availableMem;
    infos.availableRam = availableMem;
#else
    // TODO: could be done on FreeBSD too
    // see https://github.com/xbmc/xbmc/blob/master/xbmc/linux/XMemUtils.cpp
    infos.totalRam = infos.freeRam = infos.availableRam = infos.totalSwap = infos.freeSwap = std::numeric_limits<std::size_t>::max();
#endif

    return infos;
}

std::ostream& operator<<(std::ostream& os, const MemoryInfo& infos)
{
    const double convertionGb = std::pow(2, 30);
    os << std::setw(5) << "\t- Total RAM:     " << (infos.totalRam / convertionGb) << " GB" << std::endl
       << "\t- Free RAM:      " << (infos.freeRam / convertionGb) << " GB" << std::endl
       << "\t- Available RAM: " << (infos.availableRam / convertionGb) << " GB" << std::endl
       << "\t- Total swap:    " << (infos.totalSwap / convertionGb) << " GB" << std::endl
       << "\t- Free swap:     " << (infos.freeSwap / convertionGb) << " GB" << std::endl;
    return os;
}

}  // namespace system
}  // namespace aliceVision
