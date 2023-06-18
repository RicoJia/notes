========================================================================
========================================================================
### Intro
1. node js'is javascript running on the server. 
    - workflow: 
        1. sends task request to computer file system.
        2. ready to handle the next task
        3. process the task (read the file) and return the content back to client
    - Asynchronous, single threaded, non-blocking. So more memory efficient
    - Can read, write to files, manage databases

### Set up
1. `npm init` : set up `package.json`. npm is Node Package Manager
2. `npm i prompt-sync`: `i` is also for `npm install`. we install this package