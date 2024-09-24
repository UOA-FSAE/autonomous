#!/home/tanish/venvs/fix_requirements_file/bin/python3
from progress.bar import IncrementalBar
import sys
import subprocess

def install(package):
    result = subprocess.run([sys.executable, "-m", "pip", "install", "--progress-bar","on", package], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode

def main(): 
    # read in all packages
    with open("requirements.txt", 'r') as file:
        packages = [line.strip() for line in file]

    # progress bar 
    error_free_packages = []
    errored_packages = []
    pack_len = len(packages)
    with IncrementalBar('Processing', max=pack_len) as bar:
        for i,package in enumerate(packages):
            exit_code = install(package)    # install package
            if exit_code != 0:
                print(f"\npackage {package} failed at line {i+1}")
                errored_packages.append(package)
            else:
                error_free_packages.append(package)
            bar.next()

    # write new file
    with open("requirements_error_free.txt", "w") as file:
        for package in error_free_packages:
            file.write(package+"\n")
    with open("requirements_errored.txt", "w") as file:
        for package in errored_packages:
            file.write(package+"\n")

if __name__ == "__main__":
    main()