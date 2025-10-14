# Lab 3 Autograder



### 1. Install Docker
```bash
sudo apt-get update
sudo apt-get install -y docker.io
```

### 2. Setup Autograder
```bash
cd lab3-autograder
chmod +x grade.sh
```
### 3. Usage
Run Autograder
```
./grade.sh /path/to/workspace
```
There will be prompt for asking password for sudo permission.

NOTE: by running this code, the autograder will delete your buil, install and log folder under your workspace.

If you want to delete the folder that autograder created, please use
```
sudo rm -rf <replace this with folder name>
```