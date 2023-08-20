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

*/
console.log("hello world");
var arr = ["1", "2", "3"];
// This is an obj
var SomeObj = {
    a: 1,
    b: "hola"
};
// array is an subtype of object, so you can do
console.log(arr.length, SomeObj.a, typeof(SomeObj), typeof(arr));
typeof arr;

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
Bar();

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
Bar();

function Bar_let(){
    if (true){
        let i = 1;
        {
            console.log("see if let can be be accessed in nested blocks:", i);
        }
    }
}
Bar_let();

/*
Hoisting: only var has it. The variable declaration is moved to the top, not its initialization
below is equivalent to: 
    var a;
    console.log(a);
    a = 5;
    console.log(a);
let and const do not have this.
*/

var a = "42";
// can call foo() here because declaration of foo is hoisting
foo();
function foo() {
    a = 3;
    // without this, there's no hoisting, and a above will be changed
    var a;
};
console.log(a);
// this is auto global variable, DON'T DO THIS
// a; 

// Conditionals
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
    }
 }
test_use_strict() 

// character count:
console.log("character count", "character count".length);

/*
================ Functions ================
- Returning muliple values
- IFFE
- ARROW functions
*/
function return_multple_elements(){
    // this is returning an array
    // JS doesn't have tuple
    return [1,2];
}

[x,y] = return_multple_elements();
console.log(x,y);

// IIFE, immediately invoked function expression
a = test_use_strict
a();
var x = (function IIFE(){
    console.log("HELLO, IIFE!");
    return 42;
})();
// get x
x;

// Arrow functions
const dum = (req) => {
    console.log(req);
}
dum("lol");

// closures, and module
function outer(x) {
    function inner(y) {
        return x+y;
    }
    return inner;
}
console.log(outer(1)(2));

/**
 * ================ Constructors ================
 */

// use camelCase for regular functions, 
// but PascalCase for constructors, or class names
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

// A new class
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

function foo(){
    // "use strict";
    // Without use strict, this could point to 
    // 1. the outer enclosing object
    // 2. the call() method is callde on
    console.log(this.bar);
}
var obj1 = {
    foo: foo,
    bar: "bar"
};
obj1.foo(); 

var obj2 = {
    bar: "bar2"
}
foo.call(obj2);

// Prototype
var obj3 = Object.create(obj2);
obj3.baz = "baz";
console.log(obj3.bar);

// Since ES6, we have default values
function func(a=2){}
// But if you run on Pre-ES6
function func(){
   // Note: arguements is an array available to function in JS
   var a = arguments[0] !== (undefined) ? arguments[0] : 2;
   console.log(a);
}
func()

/**
 * ================ Array ================
 */

// Convert to each array element
arr.map((i) => Number(i)) // creates a new array, by iterating through each element in readCoilResult, and converting it to number

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
if (!Number.isNaN) {
    // taking advantage of the fact that NaN is the only object in JavaScript
    // that is not equal to itself
    Number.isNaN = function (x){
        console.log("using isNaN polyfill");
        return x !== x;
    }
}
Number.isNaN(123);

// Transpiler
