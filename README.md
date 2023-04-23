# autonomous

## Dev environment
### Prerequisites
- GNUmake
- VS code
  - [Remote Development Extension Pack](https://vscode.dev/github/UOA-FSAE/autonomous/blob/c73088e44093aeaae48ef29d6cef836453db4acfcode-remote-extensionpack\extension) 
- Git
- Docker (compose and engine)

### Dev environment first time setup
1. clone repo using ```git clone https://github.com/UOA-FSAE/autonomous.git```
2. cd into repo
3. run ```make build```
4. in VS Code under the Ctrl+Shift+P menu run ```>Dev Containers: attach running container to```
5. select autonomous container

### Dev environment usage
Once installed can access dev container from the remote explorer dev containers tab
1. click attach to container button (next to container name)

### Common Issues
- if have integrated graphics or more than 1 GPU may need to specify the id
