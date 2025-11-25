# RV-FSTSP

- This code using boost library. Installation in ubuntu: `sudo apt-get install libboost-all-dev`
- Build: `cmake .` then `cmake --build .`
- Model reference: https://www.overleaf.com/project/66d98809c25b3db7be64f226

## Usage

- Run command line: This will solve for 1 instance
  - Command: `./RV-FSTSP -i <input file>`
  - Additional option:
    - `-o <output dir>`: Specify where to save the output. Omit if writing output is not needed
    - `-m <3, 5>`: Solving mode. 3: Using 3-index based, 5: Using 5-index based. Default: 3 
    - `-s <0, 1, 2>`: Screen option. 0: only notify if feasibility check failed, 1: results + check, 2: all. Default: 2
    - `-t <int>`: Time limit for the solver (no limit at default)
    - `--thread=<int>`: Number of thread used. Default: Cplex choice
    - `--dtl=<double>`: Drone endurance. Default: 20
    - `--sl=<double>`: Service launch time. Default: 1
    - `--sr=<double>`: Service recovery time. Default: 1
    - `--check=<bool>`: Check the feasibility of the solution or not. Default: true
    - `--revisit=<bool>`: Allow revisiting or not. Default: true
    - `--loop=<bool>`: Allow loop or not. Default: true

[//]: # (- Using bash script: This will solve for entire directory including the instances)

[//]: # (  - Command: `./run_rv.sh` or `./run_nonrv.sh`)

[//]: # (  - Option &#40;Edit directly on variables of script file&#41;:)

[//]: # (    - `input_dir`: Directory containing **only** input files)

[//]: # (    - `output_dir`: Directory to save the output, use `skip` if writing output is not needed )

[//]: # (    - `driver`: File built from the source code)

[//]: # (    - Other parameters are the same for the solver)
    