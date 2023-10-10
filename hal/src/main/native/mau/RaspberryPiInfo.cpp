/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RaspberryPiInfo.h"

#include <stdlib.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <iostream>
#include <list>

using namespace std;

static string get_driver(const string& tty) {
    struct stat st;
    string devicedir = tty;

    // Append '/device' to the tty-path
    devicedir += "/device";

    // Stat the devicedir and handle it if it is a symlink
    if (lstat(devicedir.c_str(), &st)==0 && S_ISLNK(st.st_mode)) {
        char buffer[1024];
        memset(buffer, 0, sizeof(buffer));

        // Append '/driver' and return basename of the target
        devicedir += "/driver";

        if (readlink(devicedir.c_str(), buffer, sizeof(buffer)) > 0)
            return basename(buffer);
    }
    return "";
}

static void register_comport( list<string>& comList, list<string>& comList8250, const string& dir) {
    // Get the driver the device is using
    string driver = get_driver(dir);

    // Skip devices without a driver
    if (driver.size() > 0) {
        string devfile = string("/dev/") + basename(dir.c_str());

        // Put serial8250-devices in a seperate list
        if (driver == "serial8250") {
            comList8250.push_back(devfile);
        } else
            comList.push_back(devfile);
    }
}

static void probe_serial8250_comports(list<string>& comList, list<string> comList8250) {
    struct serial_struct serinfo;
    list<string>::iterator it = comList8250.begin();

    // Iterate over all serial8250-devices
    while (it != comList8250.end()) {

        // Try to open the device
        int fd = open((*it).c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

        if (fd >= 0) {
            // Get serial_info
            if (ioctl(fd, TIOCGSERIAL, &serinfo)==0) {
                // If device type is no PORT_UNKNOWN we accept the port
                if (serinfo.type != PORT_UNKNOWN)
                    comList.push_back(*it);
            }
            close(fd);
        }
        it ++;
    }
}

static list<string> getComList() {
    int n;
    struct dirent **namelist;
    list<string> comList;
    list<string> comList8250;
    const char* sysdir = "/sys/class/tty/";

    // Scan through /sys/class/tty - it contains all tty-devices in the system
    n = scandir(sysdir, &namelist, NULL, NULL);
    if (n < 0)
        perror("scandir");
    else {
        while (n--) {
            if (strcmp(namelist[n]->d_name,"..") && strcmp(namelist[n]->d_name,".")) {

                // Construct full absolute file path
                string devicedir = sysdir;
                devicedir += namelist[n]->d_name;

                // Register the device
                register_comport(comList, comList8250, devicedir);
            }
            free(namelist[n]);
        }
        free(namelist);
    }

    // Only non-serial8250 has been added to comList without any further testing
    // serial8250-devices must be probe to check for validity
    probe_serial8250_comports(comList, comList8250);

    // Return the lsit of detected comports
    return comList;
}

/* ----------------------------------------------------------------------- */

/*
2 2  2  2 2 2  1 1 1 1  1 1 1 1  1 1 0 0 0 0 0 0  0 0 0 0
5 4  3  2 1 0  9 8 7 6  5 4 3 2  1 0 9 8 7 6 5 4  3 2 1 0

W W  S  M M M  B B B B  P P P P  T T T T T T T T  R R R R

W  warranty void if either bit is set

S  0=old (bits 0-22 are revision number) 1=new (following fields apply)

M  0=256 1=512 2=1024

B  0=Sony 1=Egoman 2=Embest 3=Unknown 4=Embest

P  0=2835, 1=2836, 2=2837

T  0=A 1=B 2=A+ 3=B+ 4=Pi2B 5=Alpha 6=Compute Module 7=Unknown 8=Pi3B 9=Zero

R  PCB board revision

*/

static unsigned gpioHardwareRevision()
{
   static unsigned rev = 0;

   FILE * filp;
   char buf[512];
   char term;

   if (rev) return rev;

   filp = fopen ("/proc/cpuinfo", "r");

   if (filp != NULL)
   {
      while (fgets(buf, sizeof(buf), filp) != NULL)
      {
         if (!strncasecmp("revision\t:", buf, 10))
         {
            if (sscanf(buf+10, "%x%c", &rev, &term) == 2)
            {
               if (term != '\n') rev = 0;
               else rev &= 0xFFFFFF; /* mask out warranty bit */
            }
         }
      }

      fclose(filp);
   }
   return rev;
}

static std::string getRaspberryPiModelDescription(RaspberryPiModel model) {
	switch (model) {
	case RaspberryPiModel::RPI1_B_R1_0:				return "Raspberry Pi 1 Model A Rev 1.0";
	case RaspberryPiModel::RPI1_B_R2_0:				return "Raspberry Pi 1 Model B Rev 2.0";
	case RaspberryPiModel::RPI1_A_R2_0:				return "Raspberry Pi 1 Model A Rev 2.0";
	case RaspberryPiModel::RPI_CM1_R1_0:			return "Raspberry Pi Compute Module 1";
	case RaspberryPiModel::RPI_CM3_R1_0:			return "Raspberry Pi Compute Module 3";
	case RaspberryPiModel::RPI1_APLUS_R1_1:			return "Raspberry Pi 1 Model A Plus Rev 1.1";
	case RaspberryPiModel::RPI1_BPLUS_R1_0:			return "Raspberry Pi 1 Model B Plus Rev 1.0";
	case RaspberryPiModel::RPI1_BPLUS_R1_2:			return "Raspberry Pi 1 Model B Plus Rev 1.2";
	case RaspberryPiModel::RPI_ZERO_R1_2:			return "Raspberry Pi Zero Rev 1.2";
	case RaspberryPiModel::RPI_ZERO_R1_3:			return "Raspberry Pi Zero Rev 1.3";
	case RaspberryPiModel::RPI_ZEROW_R1_3:			return "Raspberry Pi Zero W Rev 1.3";
	case RaspberryPiModel::RPI2_B_R1_0:				return "Raspberry Pi 2 Model B Rev 1.0";
	case RaspberryPiModel::RPI2_B_R1_1:				return "Raspberry Pi 2 Model B Rev 1.1";
	case RaspberryPiModel::RPI3_B_R1_2:				return "Raspberry Pi 3 Model B Rev 1.2";
	case RaspberryPiModel::RPI2_B_W_BCM2837_R1_2:	return "Raspberry Pi 2 Model B Plus Rev 1.2";
	case RaspberryPiModel::RPI3_BPLUS_R1_3:			return "Raspberry Pi 3 Model B Plus Rev 1.3";
	case RaspberryPiModel::RPI3_APLUS_R1_0:			return "Raspberry Pi 3 Model A Plus Rev 1.0";
	case RaspberryPiModel::RPI4_B:				return "Raspberry Pi 4 Model B";
	default:										return "Unknown Raspberry Pi Model";
	}
}

// Reference:  https://github.com/raspberrypi/documentation/blob/master/hardware/raspberrypi/revision-codes/README.md

static RaspberryPiModel getRaspberryPiModel(unsigned hwrev) {
	// Mask off 8 highest bits
	hwrev &= 0xFFFFFF;
	if (hwrev <= 0x15) {
		switch (hwrev) {
		default:
			return RaspberryPiModel::UNKNOWN;
		case 0x2:
		case 0x3:
			return RaspberryPiModel::RPI1_B_R1_0;
		case 0x4:
		case 0x5:
		case 0x6:
			return RaspberryPiModel::RPI1_B_R2_0;
		case 0x7:
		case 0x8:
		case 0x9:
			return RaspberryPiModel::RPI1_A_R2_0;
		case 0x10:
			return RaspberryPiModel::RPI1_BPLUS_R1_0;
		case 0x11:
		case 0x14:
			return RaspberryPiModel::RPI_CM1_R1_0;
		case 0x12:
		case 0x15:
			return RaspberryPiModel::RPI1_APLUS_R1_1;
		case 0x13:
			return RaspberryPiModel::RPI1_BPLUS_R1_2;
		}
	} else {
		switch (hwrev) {
		case 0x900032:
		case 0x900021:
			return RaspberryPiModel::RPI1_APLUS_R1_1;
			break;
		case 0x900092:
		case 0x920092:
			return RaspberryPiModel::RPI_ZERO_R1_2;
			break;
		case 0x900093:
		case 0x920093:
			return RaspberryPiModel::RPI_ZERO_R1_3;
			break;
		case 0x9000c1:
			return RaspberryPiModel::RPI_ZEROW_R1_3;
		case 0xa01040:
			return RaspberryPiModel::RPI2_B_R1_0;
		case 0xa01041:
			return RaspberryPiModel::RPI2_B_R1_1;
		case 0xa02082:
		case 0xa22082:
		case 0xa32082:
		case 0xa52082:
			return RaspberryPiModel::RPI3_B_R1_2;
		case 0xa020a0:
			return RaspberryPiModel::RPI_CM3_R1_0;
		case 0xa22042:
			return RaspberryPiModel::RPI2_B_W_BCM2837_R1_2;
		case 0xa020d3:
			return RaspberryPiModel::RPI3_BPLUS_R1_3;
		case 0x9020e0:
			return RaspberryPiModel::RPI3_APLUS_R1_0;
		case 0xa03111:
		case 0xb03111:
		case 0xb03112:
                case 0xb03114:
		case 0xc03111:
		case 0xc03112:
		case 0xc03114:
		case 0xd03114:
		case 0xc03115:
		case 0xd03115:
			return RaspberryPiModel::RPI4_B;
		default:
			return RaspberryPiModel::UNKNOWN;
		}
	}
}

// Reference:  Wikipedia article on "Raspberry Pi"

static uint32_t getRaspberryPiRamMB(unsigned hwrev) {
	uint32_t ram_mb = 0;
	// Mask off 8 highest bits
	hwrev &= 0xFFFFFF;
	if (hwrev <= 0x15) {
		switch (hwrev) {
		default:
		case 0x2:
		case 0x3:
		case 0x4:
		case 0x5:
		case 0x6:
		case 0x7:
		case 0x8:
		case 0x9:
		case 0x12:
			ram_mb = 256;
			break;
		case 0x10:
		case 0x11:
		case 0x13:
		case 0x14:
			ram_mb = 512;
			break;
		}
	} else {
		hwrev >>= 20;
		hwrev &= 0x07;
		switch (hwrev) {

		case 0:
			ram_mb = 256;
			break;
		default:
		case 1:
			ram_mb = 512;
			break;
		case 2:
			ram_mb = 1024;
			break;
		case 3:
			ram_mb = 2048;
			break;
		case 4:
			ram_mb = 4096;
			break;
		case 5:
			ram_mb = 8192;
			break;
		}
	}
	return ram_mb;
}

// Reference:  Wikipedia article on "Raspberry Pi"

static bool getRaspberryPiHas40PinGpio(unsigned hwrev) {

	RaspberryPiModel model = getRaspberryPiModel(hwrev);

	switch (model) {
	default:
	case RaspberryPiModel::UNKNOWN:
	case RaspberryPiModel::RPI1_B_R1_0:
	case RaspberryPiModel::RPI1_B_R2_0:
	case RaspberryPiModel::RPI1_A_R2_0:
		return false;
	case RaspberryPiModel::RPI_CM1_R1_0:
	case RaspberryPiModel::RPI_CM3_R1_0:
	case RaspberryPiModel::RPI1_APLUS_R1_1:
	case RaspberryPiModel::RPI1_BPLUS_R1_0:
	case RaspberryPiModel::RPI1_BPLUS_R1_2:
	case RaspberryPiModel::RPI_ZERO_R1_2:	// Unpopulated, requires user to solder the header
	case RaspberryPiModel::RPI_ZERO_R1_3:	// Unpopulated, requires user to solder the header
	case RaspberryPiModel::RPI_ZEROW_R1_3:	// Unpopulated, requires user to solder the header
	case RaspberryPiModel::RPI2_B_R1_0:
	case RaspberryPiModel::RPI2_B_R1_1:
	case RaspberryPiModel::RPI3_B_R1_2:
	case RaspberryPiModel::RPI2_B_W_BCM2837_R1_2:
	case RaspberryPiModel::RPI3_BPLUS_R1_3:
	case RaspberryPiModel::RPI3_APLUS_R1_0:
	case RaspberryPiModel::RPI4_B:
		return true;
	}
	return false;
}

// Reference:  Wikipedia article on "Raspberry Pi"

static int getRaspberryPiNumUSBPorts(unsigned hwrev)
{
	RaspberryPiModel model = getRaspberryPiModel(hwrev);

	switch (model) {
	case RaspberryPiModel::RPI1_B_R1_0:				return 2;
	case RaspberryPiModel::RPI1_B_R2_0:				return 2;
	case RaspberryPiModel::RPI1_A_R2_0:				return 1;
	case RaspberryPiModel::RPI_CM1_R1_0:			return 1;
	case RaspberryPiModel::RPI_CM3_R1_0:			return 1;
	case RaspberryPiModel::RPI1_APLUS_R1_1:			return 1;
	case RaspberryPiModel::RPI1_BPLUS_R1_0:			return 4;
	case RaspberryPiModel::RPI1_BPLUS_R1_2:			return 4;
	case RaspberryPiModel::RPI_ZERO_R1_2:			return 1;
	case RaspberryPiModel::RPI_ZERO_R1_3:			return 1;
	case RaspberryPiModel::RPI_ZEROW_R1_3:			return 1;
	case RaspberryPiModel::RPI2_B_R1_0:				return 4;
	case RaspberryPiModel::RPI2_B_R1_1:				return 4;
	case RaspberryPiModel::RPI3_B_R1_2:				return 4;
	case RaspberryPiModel::RPI2_B_W_BCM2837_R1_2:	return 4;
	case RaspberryPiModel::RPI3_BPLUS_R1_3:			return 4;
	case RaspberryPiModel::RPI3_APLUS_R1_0:			return 1;
	case RaspberryPiModel::RPI4_B:				return 4;
	default:										return 0;
	}
}

// Port identification and discrimination is based upon emprical analyses of live boards by Kauai Labs.

static RaspberryPiUSBPort getRaspberryPIUSBPort(unsigned hwrev, std::string short_usb_device_id) {
	RaspberryPiModel model = getRaspberryPiModel(hwrev);
	int num_usb_ports = getRaspberryPiNumUSBPorts(hwrev);

	if (num_usb_ports == 0) {
		return RaspberryPiUSBPort::INVALID;
	} else if (num_usb_ports == 1) {
		return RaspberryPiUSBPort::RPI_USB_PORTA;
	}

	// The remaining models have either 2 or 4 ports, in which case discrimination is required.

	switch(model) {
		case RaspberryPiModel::RPI3_BPLUS_R1_3:
			if (short_usb_device_id.rfind("1-1.1.2") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTA;
			} else if (short_usb_device_id.rfind("1-1.1.3") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTB;
			} else if (short_usb_device_id.rfind("1-1.3") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTC;
			} else if (short_usb_device_id.rfind("1-1.2") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTD;
			} else {
				return RaspberryPiUSBPort::INVALID;
			}
			break;
		case RaspberryPiModel::RPI3_B_R1_2:
			if (short_usb_device_id.rfind("1-1.2") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTA;
			} else if (short_usb_device_id.rfind("1-1.3") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTB;
			} else if (short_usb_device_id.rfind("1-1.4") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTC;
			} else if (short_usb_device_id.rfind("1-1.5") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTD;
			} else {
				return RaspberryPiUSBPort::INVALID;
			}
			break;
		case RaspberryPiModel::RPI4_B:
			if (short_usb_device_id.rfind("1-1.3") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTA;
			} else if (short_usb_device_id.rfind("1-1.4") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTB;
			} else if (short_usb_device_id.rfind("1-1.1") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTC;
			} else if (short_usb_device_id.rfind("1-1.2") == 0) {
				return RaspberryPiUSBPort::RPI_USB_PORTD;
			} else {
				return RaspberryPiUSBPort::INVALID;
			}
			break;
		default:
			return RaspberryPiUSBPort::INVALID;
	}
}

list<USBSerialDeviceInfo> getRaspberryPiUSBSerialDeviceInfoList() {
    list<string> l = getComList();

    list<string>::iterator it = l.begin();
    DIR *d;
    struct dirent *dir;
    list<USBSerialDeviceInfo> usbser_devinfo_list;
    unsigned hwrev = gpioHardwareRevision();

    while (it != l.end()) {
	// Print out all except the built-in raspberry pi serial ports
	if ((*it).compare("/dev/ttyS0") && (*it).compare("/dev/ttyAMA0")) {
		// Extract the USB Path

		// Example USB Device Paths on Raspberry Pi 3B+:
		// ../../devices/platform/soc/3f980000.usb/usb1/1-1/1-1.2/1-1.2:1.0/tty/ttyACM0
		// Example USB Device Paths on Raspberry Pi 4B:
		// .../../devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.3/1-1.3.4/1-1.3.4:1.0/tty/ttyACM0
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*****<<<<<<<<<<<<<<<<
		// The portion denoted by asterisk ("*") uniquely identifies one of four USB connectors on a RPI3B+

		string classfile = string("/sys/class/tty/") + basename((*it).c_str());
		char physical_device_path[1024];
		char physical_device_id[1024];
		std::string devpath;
		RaspberryPiUSBPort physical_usb_port = RaspberryPiUSBPort::INVALID;
		ssize_t pathlen = readlink(classfile.c_str(), physical_device_path, sizeof(physical_device_path));
       		if (pathlen != -1) {
			physical_device_path[pathlen] = 0;
			devpath = physical_device_path;
			if (devpath.find("usb") != std::string::npos) {
				size_t usbpos = devpath.find("usb/");
				if (usbpos != std::string::npos) {
					devpath.erase(0, usbpos+3);
					size_t ttypos = devpath.find("/tty");
					if (ttypos != std::string::npos) {
						devpath.erase(ttypos);
					}
					size_t finalslashpos = devpath.rfind("/");
					if (finalslashpos != std::string::npos) {
						devpath.erase(0, finalslashpos+1);
					}
					size_t finalcolonpos = devpath.rfind(":");
					if (finalslashpos != std::string::npos) {
						devpath.erase(finalcolonpos);
					}
					physical_usb_port = getRaspberryPIUSBPort(hwrev, devpath);
				} else {
					// Starting with Raspberry Pi Model 4B, the string "usb1"
					// is present in the usb device path strings
					size_t usbpos = devpath.find("usb1/");
					if (usbpos != std::string::npos) {
						devpath.erase(0, usbpos+4);
						size_t ttypos = devpath.find("/tty");
						if (ttypos != std::string::npos) {
							devpath.erase(ttypos);
						}
						size_t finalslashpos = devpath.rfind("/");
						if (finalslashpos != std::string::npos) {
							devpath.erase(0, finalslashpos+1);
						}
						size_t finalcolonpos = devpath.rfind(":");
						if (finalslashpos != std::string::npos) {
							devpath.erase(finalcolonpos);
						}
						cout << "Found tty devpath:  " << devpath;
						physical_usb_port = getRaspberryPIUSBPort(hwrev, devpath);
					}
				}
			}
		}
		// Retrieve Device ID
		d = opendir("/dev/serial/by-id");
		if (d) {
			while ((dir = readdir(d)) != NULL) {
				char fullentrypath[2048];
				strcpy(fullentrypath, "/dev/serial/by-id/");
				strcat(fullentrypath, dir->d_name);
				ssize_t entrypathlen = readlink(fullentrypath, physical_device_id, sizeof(physical_device_id));
				if (entrypathlen != -1) {
					physical_device_id[entrypathlen] = 0;
					if (strstr(physical_device_id, basename((*it).c_str())) != NULL) {
						usbser_devinfo_list.push_back(USBSerialDeviceInfo((*it), devpath, physical_device_path, physical_device_id, physical_usb_port));
						break;
					}
				}
			}
		}
		closedir(d);
	}
        it++;
    }
    return usbser_devinfo_list;
}

RaspberryPiDeviceInfo getRaspberryPiDeviceInfo() {
    unsigned hwrev = gpioHardwareRevision();
    RaspberryPiModel model = getRaspberryPiModel(hwrev);
    std::string model_description = getRaspberryPiModelDescription(model);
    uint32_t ram_mb = getRaspberryPiRamMB(hwrev);
    bool has40PinGPIO = getRaspberryPiHas40PinGpio(hwrev);
    int num_usb_ports = getRaspberryPiNumUSBPorts(hwrev);
    return RaspberryPiDeviceInfo(model, model_description, hwrev, ram_mb, has40PinGPIO, num_usb_ports);
}

#if 0
int main() {

	// Acquire/Display RPI Hardware Info
	RaspberryPiDeviceInfo rpi_info = getRaspberryPiDeviceInfo();
    cout << "Raspberry Pi Hardware Revision:  " << std::hex << rpi_info.hwrev << std::dec << endl;

    cout << "Model:  " << rpi_info.model_description << endl;
    cout << "RAM (MB):  " << rpi_info.ram_mb << endl;
    cout << "USB Ports:  " << rpi_info.num_usb_ports << endl;
    if (rpi_info.has_40pin_gpio) {
    	cout << "GPIO:  40 Pin GPIO Supported";
    } else {
    	cout << "GPIO:  INCOMPATIBLE WITH 40 Pin GPIO Support!";
    }
    cout << endl;

    // Acquire/Display RPI USB Serial Port Info (note this is dynamic, and changes as ports are plugged in/out).
    list<USBSerialDeviceInfo> usbser_devinfo_list = getRaspberryPiUSBSerialDeviceInfoList();
    list<USBSerialDeviceInfo>::iterator itdil = usbser_devinfo_list.begin();
    while (itdil != usbser_devinfo_list.end()) {
        cout << endl;
		cout << "Name:           " << (*itdil).devname << endl;
		cout << "USB ShortPath:  " << (*itdil).usbshortpath << endl;
		cout << "USB LongPath:   " << (*itdil).usblongpath << endl;
		cout << "DeviceID:       " << (*itdil).deviceid << endl;
		switch ((*itdil).physical_usb_port) {
			case RaspberryPiUSBPort::INVALID:		cout << "Physical USB Port:  INVALID" << endl; break;
			case RaspberryPiUSBPort::RPI_USB_PORTA:	cout << "Physical USB Port:  A (left-top)" << endl; break;
			case RaspberryPiUSBPort::RPI_USB_PORTB:	cout << "Physical USB Port:  B (left-bottom)" << endl; break;
			case RaspberryPiUSBPort::RPI_USB_PORTC:	cout << "Physical USB Port:  C (right-top)" << endl; break;
			case RaspberryPiUSBPort::RPI_USB_PORTD:	cout << "Physical USB Port:  D (right-bottom)" << endl; break;
		}
		itdil++;
    }

    return 0;
}
#endif
