import argparse
parser = argparse.ArgumentParser()
# interesting, -- does make a difference
parser.add_argument("--client", type=bool, default=False, required=False)
args = parser.parse_args()
if args.client:
    print("client")
else:
    print("server")
