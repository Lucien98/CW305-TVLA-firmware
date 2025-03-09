import chipwhisperer as cw


def parse_arguments():
	import argparse

	parser = argparse.ArgumentParser()
	parser.add_argument('--order', metavar='order', required=False, default=2,
							help='program the firmware according to the order', type=int)
	parser.add_argument('--no_initial_sharing', action="store_true",
							help='program the firmware that do not give initial sharing to key and pt')
	args = parser.parse_args()
	return args

def main():
	args = parse_arguments()
	postfix = ""
	if args.no_initial_sharing and args.order > 0:
		postfix = "_nosharing"
	print(" You need first to short J5 and reset\n\n")
	fw_path = "../../hardware/victims/cw305_artixtarget/fw/sam3u/CW305_SAM3U_FW/build/CW305_SAM3UFWo%d%s.bin" % (args.order, postfix)
	programmer = cw.SAMFWLoader(scope=None)
	programmer.program("COM7", fw_path)
	print("\n\n Reset the target again")

if __name__ == '__main__':
	main()
