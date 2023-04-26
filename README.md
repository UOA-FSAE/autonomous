# autonomous

## Dev environment
### Prerequisites
- GNUmake
  - can install on windows using [chocolatey](https://chocolatey.org/install#generic)
- VS code
  - [Remote Development Extension Pack](https://vscode.dev/github/UOA-FSAE/autonomous/blob/c73088e44093aeaae48ef29d6cef836453db4acfcode-remote-extensionpack\extension) 
- Git
- Docker (compose and engine)

### Dev environment first time setup
1. clone repo using ```git clone https://github.com/UOA-FSAE/autonomous.git```
2. cd into repo
3. run ```make build```
   - use cmd if on windows
4. in VS Code under the Ctrl+Shift+P menu run ```>Dev Containers: attach running container to```
5. select autonomous container

### Dev environment usage
After first time setup can access dev container from the remote explorer dev containers tab
1. click attach to container button (next to container name)

