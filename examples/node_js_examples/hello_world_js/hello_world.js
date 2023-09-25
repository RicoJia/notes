//  "use strict";
/*
================ Variables ================
- types (object)
    - typed values, not typed variables.
    - Null, object, symbol, symbols (ES6)
- typeof
- coercion
- inequality
- let vs var:
    - let is block scoped, var is function scoped
    - let cannot be redefined, var can 
    - Both are hoisted, but let is not initialized. var is initialized
- Immutables: bool, string, int are all immutables. Objects are mutables
*/

const test_coercion = () => {
    // coercion
    var a = "42";
    // typeof null returns object. This is a bug that likely not gonna get fixed.
    // 1=='1' is true in JS because type cocersion is done here
    // 1===1 is "strict equality"
    console.log(Number(a), typeof null, 1===1, 1=='1');

    // Inequality: there's no strict inequality.
    // see 3 falses, becasue "foo" is coerced into 'NaN'
    console.log(42<"foo", 42 == "foo", 42 > "foo");
    // Comparison operators will compare all of them into numbers
    console.log("foo"<42, "foo">42);
}

/*
scope - or lexical scope. That is, where in the code a variable is defined
1. Different from C++, any variable in the current scope can be accessed elsewhere in the scope
2. Hoisting. The term means "raising by pulley". That is, declaration of a variable
will move the variable, function to the top of the current scope
Be aware
3. There are 3 scopes:
    - global scope
    - function scope (var)
    - block scope (const, and let)
*/

// Scope. Where as in C++, i will have block scope
function Bar(){
    if (true){
        var i = 1;
    }
    console.log(i);
}
// Bar();

// let has block scope, a block is a pair of {}
function Bar(){
    if (true){
        let i = 1;
    }

    try{
        // here will see ReferenceError: i is not defined
        console.log(i);
    } catch(error){
    }
    
}
// Bar();

function Bar_let(){
    if (true){
        let i = 1;
        {
            console.log("see if let can be be accessed in nested blocks:", i);
        }
    }
}
// Bar_let();

/*
Hoisting: 
    - const and let does not have hoisting
    - var: the variable declaration is moved to the top of the function, so you will see "undefined" before
    var gets initialized.
    - function: the function declaration and definition both are hoisted. 
        IMPORTANT: hoisting makes functions visible throughout the current scope!
*/
const test_hoising = () => {
    console.log('Test hoisting: before declaration, should see undefined: ', a);
    var a = "42";
    console.log("testing hoisting, after declaration: ", a);
    foo();
    function foo() {
        a = 3;
        // without this, there's no hoisting, and a above will be changed
        // var a;
        console.log("testing function hoisting: ", a);
    };
}
// test_hoising()


// Conditionals
const test_conditionals = () => {
    var a;
    if (false){}
    else if (true){}

    switch (a){
        case 2:
            break;
        case 3:
            break;
        default:
            break;
    }

    var dummy = (true)? 1: 1;
    
}

// character count:
const test_character_count = () => {
    console.log("character count", "character count".length);
}

// Object Destructing
const test_function_destructing = () => {
    let constants = {
        MAPS: "maps",
        TILES: "tiles"
    }
    let {MAPS, DOORS} = constants;
    console.log("test function destructing: ", MAPS, DOORS);
}
test_function_destructing();

const test_multiple_declarations = () => {
    // declaring a buch of variables
    let var1, var2 = {}, var3 = 42;
}

/*
================ Functions ================
- Returning muliple values
- IFFE
- ARROW functions
*/
const test_return_multiple_values = () => {
    function return_multple_elements(){
        // this is returning an array
        // JS doesn't have tuple
        return [1,2];
    }

    [x,y] = return_multple_elements();
    console.log("test return multiple values: ",x,y);
}
test_return_multiple_values();

// strict mode - "use strict" pragma is placed in the scope of your program
// You have to do it at the top of your program, or the function.
// Anywhere below won't do the trick. 
 function test_use_strict() {
    "use strict";
    try{
        secondVariable = "new-test";
        firstVariable = "test";
        firstVariable = "test2";
        console.log(secondVariable);   
    } catch(e){
        console.log("testing use strict, should see unreferenced error: ", e);
    }
 }
test_use_strict()

// IIFE, immediately invoked function expression
const test_iife = () => {
    var x = (function IIFE(){
        console.log("HELLO, IIFE!");
        return 42;
    })();
    // get x
    x;
}

const test_closures = () => {
    // closures, and module
    function outer(x) {
        function inner(y) {
            return x+y;
        }
        return inner;
    }
    console.log(outer(1)(2));
}

/**
 * ================ Constructors ================
 */

// use camelCase for regular functions, 
// but PascalCase for constructors, or class names
const test_module_pattern = () =>{
    function SomeModule(){
        var pwd, name;
        function login(n, pw) {
            name = n.toLowerCase();
            pwd = pw
            console.log("name:", name, "pwd:", pwd);
        }
        var publicAPI = {
            login: login
        };
        return publicAPI;
    }

    var fred = SomeModule();
    fred.login("fred", "12345");
}


/*
This, property, class, constructor function, prototype
- Constructor function: when used w/ new
    - Creates a new JS object
    - Sets properties in this new object
- This
- Prototype seems to be the "class variable" cluster. 
    - Not just that, prototype is the mechanism to fake "inheritance" from other classes
    - or delegate to other objects.
    - If you can't find a prototype in a object, JS will find in objects it delegates to
*/
const test_ctor_function = () => {
    function Car(name){
        // Can detect if being called with new. This will be undefined if not called with new
        if (!new.target){
            // note, string interpolation can only be done with `
            return `${name} should be a car. Please call with new`;
        }
        else{
            this.name = name;
            console.log(`New car ${this.name} is created`);
        }
    }

    funny_car = Car("funny car");
    console.log(funny_car, typeof(funny_car));
    var funny_car = new Car("funny car");
    // prototype
    Car.prototype.color = "red";
    // You can add more properties as well
    funny_car.engine = "Google Chrome V8";
    console.log(funny_car.color, funny_car.engine);
}

// A new class
const test_class = () => {
    class Carro{
        constructor(name){
            this.name = name;
        }
        meep(){
            console.log("Herro");
        }
    }
    // must call with new, otherwise, see an ReferenceError
    var c = new Carro("c");
    c.meep();
    
}

// Prototype
const test_create_prototype = () => {
    var obj3 = Object.create(obj2);
    obj3.baz = "baz";
    console.log("creating an object prototype: ", obj3.bar);
}
// test_create_prototype();

const test_default_value = () => {
    // Since ES6, we have default values
    function func(a=2){}
    // But if you run on Pre-ES6
    function func(){
    // Note: arguements is an array available to function in JS
    var a = arguments[0] !== (undefined) ? arguments[0] : 2;
    console.log(a);
    }
    func()
}


/**
 * ================ Browser vs Node ================
 * Many things are exposed to JS, but not implemented. They are called "host objects"
- On browser, 
    - alert("adsfasd"), console.log(), they are implemented in C/C++, exposed to JS.
        - Now they could be implemented in JS
    - DOM, document.getElementById(), is implemented in browser as well
    - best practices In HTML:
        - CSS stylesheet always at top, because it affects styles of subsequent elements
        - JS scripts are included at the bottom, because this way you know all elements required are already there.
    - The purpose of DOM:
        - A tree of objects, that can be used to dynamically change the page (clicking buttons, etc.)
- In node.js:
    - Process, fs, http
- Native JS objects: 
    - types: Array, Strings
    - Math, RegExp
 */

// On browser, can use $0 to refer to the currently inspected element

/*
 * ================ Transpiler and Polyfill ================
*/

/*
- Polyfill - piece of code on old browser to fake BEHAVIOR of some new features.
    - If you run the same code on a new browser, then that code has no effect
    E.g., Number.isNaN, replaces the older isNaN, which is more buggy
    Or, fetch() is better than XMLHttpRequest. You can use fetch() on all browsers, but you 
    have to implement using XMLHttpRequest
- Transpiler: piece of code run on older browser, to fake SYNTAX of a newer code
*/
// this check is very necessary 
const test_polyfill = () => {
    if (!Number.isNaN) {
        // taking advantage of the fact that NaN is the only object in JavaScript
        // that is not equal to itself
        Number.isNaN = function (x){
            console.log("using isNaN polyfill");
            return x !== x;
        }
    }
    Number.isNaN(123);
}
test_polyfill();

const test_try_catch_finally = () => {
// JS doesn't have `try-catch-else`. Only `try-catch-finally`
}
