## JS project: 
- add a new page: ```functions.php```, look for wordpress_post
- pages are stored in MySQL database. phpmyadmin

### Concepts
- IIFE (Immediately Invoked Function Expressions)
```python
(function (){console.log("hehe")})()
```
- `const` was introduced in ES6
- DTO (datoa transfer objects), used for slimming down a data object before sending them over wire

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


### Syntax: TODO experiment
1. import vs export
    ```
    
    ```
    ```
    import myLocalFunction from './mylocalmodule';
    // Global modules are in node_modules, from ES6
    import function from 'module';
    // older version:
    const express = require('express')
    // for an exported member
    import { myLocalFunction } from './mylocalmodule';
    ```
