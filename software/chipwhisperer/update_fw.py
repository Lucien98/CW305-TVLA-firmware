import chipwhisperer as cw


def parse_arguments():
	import argparse

	parser = argparse.ArgumentParser()
	parser.add_argument('--order', metavar='order', required=False, default=2,
							help='program the firmware according to the order', type=int)
	args = parser.parse_args()
	return args

def main():
	args = parse_arguments()
	print(" You need first to short J5 and reset\n\n")
	fw_path = "../../hardware/victims/cw305_artixtarget/fw/sam3u/CW305_SAM3U_FW/build/CW305_SAM3UFWo%d.bin" % args.order
	programmer = cw.SAMFWLoader(scope=None)
	programmer.program("COM7", fw_path)
	print("\n\n Reset the target again")

if __name__ == '__main__':
	main()
