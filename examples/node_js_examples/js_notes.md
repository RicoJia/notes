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

### Event loop
Event loop is a mechanism that poll-checks queues of events, then execute them one by one. In JS, 
    - Callstack: stack (LIFO) that pops functions off for execution.
    - Libuv (node JS): executes async futures in C++ threads.
    - Task queues:
        - macro tasks (e.g., callbacks from setTimeOut)
        - microtasks: when Async futures are done from libuv, callbacks comes here.
    - Event loop: constantly pushes items the micro and macro task queues onto the callstack

E.g.,
    ```
    console.log('Start');
    setTimeout(function() {
      console.log('setTimeout');
    }, 1000);

    Promise.resolve().then(function() {
      console.log('promise1');
    });

    Promise.resolve().then(function() {
      console.log('promise2');
    });
    setTimeout(function() {
      console.log('setTimeout2');
    }, 500);
    ```
    1. console.log("start") gets on to the callstack. Then, it's popped for execution
    1. setTimeout(...) gets on to the callstack, and popped for execution.
        1. console.log('setTimeout');  onto macro tasks queue, and scheduled at 1000ms later
    1. Promise.resolve() executes (on libuv)
        1. callback gets put onto the micro-task queue
    1. Event loop detects that callback is ready for execution, so it pushes it onto callstack. Then, callback gets executed.
    1. Promise.resolve() executes while its future is executed on libuv. Then `console.log('promise2');` gets put onto microtasks queue, then gets onto callstack, then executes
    1. setTimeout(...) gets on to the callstack, and popped for execution.
        1. console.log('setTimeout2'); gets onto macro tasks queue, before the first timeout callback

