#include <stdio.h>
#include <stdlib.h>
#include <strsafe.h>
#include <windows.h>
#include <setupAPI.h>
#include <INITGUID.H>
#include <winIoCtl.h>
#include <assert.h>

#include "xdma_public.h"

#pragma comment(lib, "SetupAPI.lib")
int get_devices(GUID guid, char *devpath, size_t len_devpath)
{
        HDEVINFO device_info = SetupDiGetClassDevs(
            (LPGUID)&guid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (device_info == INVALID_HANDLE_VALUE)
        {
                fprintf(stderr, "GetDevices INVALID_HANDLE_VALUE\n");
                exit(-1);
        }

        SP_DEVICE_INTERFACE_DATA device_interface;
        device_interface.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        // enumerate through devices
        DWORD index;
        for (index = 0; SetupDiEnumDeviceInterfaces(device_info, NULL,
                                                    &guid, index, &device_interface);
             ++index)
        {
                // get required buffer size
                ULONG detailLength = 0;
                if (!SetupDiGetDeviceInterfaceDetail(device_info, &device_interface, NULL,
                                                     0, &detailLength, NULL) &&
                    GetLastError() != ERROR_INSUFFICIENT_BUFFER)
                {
                        fprintf(stderr, "SetupDiGetDeviceInterfaceDetail - get length failed\n");
                        break;
                }

                // allocate space for device interface detail
                PSP_DEVICE_INTERFACE_DETAIL_DATA dev_detail =
                    (PSP_DEVICE_INTERFACE_DETAIL_DATA)HeapAlloc(
                        GetProcessHeap(), HEAP_ZERO_MEMORY, detailLength);
                if (!dev_detail)
                {
                        fprintf(stderr, "HeapAlloc failed\n");
                        break;
                }
                dev_detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

                // get device interface detail
                if (!SetupDiGetDeviceInterfaceDetail(device_info, &device_interface,
                                                     dev_detail, detailLength, NULL, NULL))
                {
                        fprintf(stderr, "SetupDiGetDeviceInterfaceDetail - get detail failed\n");
                        HeapFree(GetProcessHeap(), 0, dev_detail);
                        break;
                }
                StringCchCopy(devpath, len_devpath, dev_detail->DevicePath);
                HeapFree(GetProcessHeap(), 0, dev_detail);
        }

        SetupDiDestroyDeviceInfoList(device_info);

        return index;
}

void CleanupDevice(HANDLE device) { CloseHandle(device); }

HANDLE open_device(char device_base_path[], const char *device_name,
                   LARGE_INTEGER base_address)
{
        char device_path[MAX_PATH + 1] = "";
        strcpy_s(device_path, sizeof device_path, device_base_path);
        strcat_s(device_path, sizeof device_path, "\\");
        strcat_s(device_path, sizeof device_path, device_name);

        // open device file
        HANDLE device = CreateFile(device_path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                   OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

        // HANDLE device = CreateFile;
        if (device == INVALID_HANDLE_VALUE)
        {
                fprintf(stderr, "Error opening device, win32 error code: %ld\n", GetLastError());
        }

        return device;
}

BYTE *allocate_buffer(size_t size, size_t alignment)
{
        if (size == 0)
        {
                size = 4;
        }
        if (alignment == 0)
        {
                SYSTEM_INFO sys_info;
                GetSystemInfo(&sys_info);
                alignment = sys_info.dwPageSize;
        }

        return (BYTE *)_aligned_malloc(size, alignment);
}