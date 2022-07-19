#include <asio.hpp>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
	asio::io_context io_context;
	asio::posix::stream_descriptor stream(io_context,
	                                      ::socket(PF_CAN, SOCK_RAW, CAN_RAW));
	struct ifreq ifr;
	strcpy(ifr.ifr_name, "vcan0");
	ioctl(stream.native_handle(), SIOCGIFINDEX, &ifr);
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	// https://github.com/linux-can/can-utils/blob/master/cansend.c
	// switch to CAN FD mode
	int enable_canfd = 1;
	if (setsockopt(stream.native_handle(), SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
	               &enable_canfd, sizeof(enable_canfd))) {
		printf("error when enabling CAN FD support\n");
		return 1;
	}
	bind(stream.native_handle(), (struct sockaddr*)&addr, sizeof(addr));

	struct canfd_frame frame = {
	    .can_id = 0x12345678 | CAN_EFF_FLAG,
	    .len = 64,
	    .flags = 3,
	    .__res0 = 0,
	    .__res1 = 0,
	    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
	             0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,
	             0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
	             0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
	             0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32,
	             0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c,
	             0x3d, 0x3e, 0x3f, 0x41}};
	// write(stream.native_handle(), &frame, sizeof(frame));
	stream.write_some(asio::buffer(&frame, sizeof(frame)));

	struct canfd_frame rframe;
	// read(stream.native_handle(), &rframe, sizeof(rframe));
	stream.read_some(asio::buffer(&rframe, sizeof(rframe)));
	if (rframe.can_id & CAN_EFF_FLAG) {
		printf("RX %08X ", rframe.can_id & CAN_EFF_MASK);
	} else {
		printf("RX %03X ", rframe.can_id & CAN_SFF_MASK);
	}
	printf("%c %c [%d]", (rframe.flags & CANFD_BRS) ? 'B' : '-',
	       (rframe.flags & CANFD_ESI) ? 'E' : '-', rframe.len);
	for (int i = 0; i < rframe.len; i++) {
		printf(" %02X", rframe.data[i]);
	}
	printf("\n");

	io_context.run();

	return 0;
}