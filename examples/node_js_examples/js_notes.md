## JS project: 
- add a new page: ```functions.php```, look for wordpress_post
- pages are stored in MySQL database. phpmyadmin

### Concepts
- IIFE (Immediately Invoked Function Expressions)
```python
(function (){console.log("hehe")})()
```
- `const` was introduced in ES6

========================================================================
## Node Js
========================================================================
### Intro
1. node js'is javascript runtime.
    - workflow: 
        1. sends task request to computer file system.
        2. ready to handle the next task
        3. process the task (read the file) and return the content back to client
    - Asynchronous, single threaded, non-blocking. So more memory efficient
    - Can read, write to files, manage databases

### Set up
1. run a simple script: `node script`
1. If you have an node js project, that depends on other npm libraries, then, create a new project
    1. `npm init` : set up `package.json`. npm is Node Package Manager
    2. `npm i prompt-sync`: `i` is also for `npm install`. we install this package