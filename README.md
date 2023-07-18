# Solver of Q-Coverage and Q-Connectivity

1. Firstly, open `cmd` and run command `pip install -r requirements.txt` to install necessary packages.
2. Run `python main.py` with different flags to customize the generated data.
- `--N`: number of targets
- `--qm`: maximum q of all the targets (default = 5)
- `--rs`: sensing radius (default = 40)
- `--rc`: communication radius (default = 80)
- `--rcl`: cluster radius (default = 80) (note that rcl >= 2rs)
- `--ra`: random q or not (boolean) (default = True)
- `--a`: area width (=height) (default = 1000)
- `--b`: base station location (default = [0, 0])

For example: `python main.py --N 100 --qm 5 --rs 30 --rc 80 --rcl 80 --ra True --b 10,10`

The output includes: target locations, base location, sensor locations and relay locations. All of them are dumped into a json file `out.json`. The visualization of the results is also created.
