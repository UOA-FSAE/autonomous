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
   - use git bash if on windows
4. in VS Code under the Ctrl+Shift+P menu run ```Dev Containers: Rebuild and Reopen in Container```
   - or if prompted click reopen in container from popup

### Normal usage
#### Method 1
1. Go to remote explorer
2. Click icon next to autonomous devcontainer (open folder in container)

#### Method 2
1. in VS Code under the Ctrl+Shift+P menu run ```Dev Containers: Rebuild and Reopen in Container``
   - with folder open in vscode
