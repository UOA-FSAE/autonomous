# UOA FSAE - autonomous

## Dev environment
### Prerequisites
- GNUmake
  - install on windows using [chocolatey](https://chocolatey.org/install#generic)
   - install on mac using homebrew ```brew install make```
   - install on linux using apt ```apt install make```
- VS code
  - [Remote Development Extension Pack](https://vscode.dev/github/UOA-FSAE/autonomous/blob/c73088e44093aeaae48ef29d6cef836453db4acfcode-remote-extensionpack\extension) 
- Git
  - [install link](https://git-scm.com/downloads) 
- Docker (compose and engine)
   - [install link](https://docs.docker.com/get-docker/)
### Local dev environment - first time setup
1. clone repo using ```git clone https://github.com/UOA-FSAE/autonomous.git```
2. cd into repo
3. run ```make build```
   - use git bash if on windows
4. in VS Code under the Ctrl+Shift+P menu run ```Dev Containers: Rebuild and Reopen in Container``` or run ```make start```
   - or if prompted click reopen in container from popup

### Local dev environment - normal usage
#### Method 1
1. Go to remote explorer
2. Click icon next to autonomous devcontainer (open folder in container)

#### Method 2
1. In VS Code under the Ctrl+Shift+P menu run ```Dev Containers: Rebuild and Reopen in Container```
   - with folder open in vscode

### Codespaces dev environment
1. Go to remote explorer
2. Change to GitHub Codespaces tab
3. Click connect to codespace icon next to desired codespace

## Ros2
### Running commands
- use ```ros2 <command>```