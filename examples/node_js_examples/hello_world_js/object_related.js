"use strict";
/*
Objects basics: 
    - using {}. So {} has 2 meanings (block scope, and object)
    - if a name is not valid for binding, double quote them. Else no need
    - Way to get all keys: Object.keys(obj)
    - Each object, and array inside objects are stored as a reference.
        - except for primitive types, like BigInt, Number
    - obj[property] is equivalent to obj.property.
OBJ is not a map, keys should be easily transformed into strings 
*/
const test_object_basics = () => {
    var obj = {
        somProperty: "haha",
        "a": 1
    }

    console.log("keys of obj", Object.keys(obj), "type of object: ", typeof(obj));

    // adding a new property, then use "in" to see if it's in the object. (hasattr in python)
    obj.someNewProperty = "new property";
    console.log("new property", obj.someNewProperty, "in: ", "someNewProperty" in obj);
    console.log('Or access the property using []: ', obj["someNewProperty"]);

}
// test_object_basics();

/*
 * ================ Prototype ================
Prototypes of JS. It's central to JS. Instead of using classes
Every object has a prototype object, and every prototype object has a prototype ... All the way 
until Object.prototype. A prototype object is just {method1: ...}
When you create a regular object, you can do:
- Object.create(prototype_object) to create an object with prototype_object being the prototype object
- new Ctor(), where after the ctor() is called, the function's prototype object becomes 
    
Basic uses
    - If you can't find a method, it will search in the prototype
    - There's a prototype chain. Similar to Python's MRO (method resolution order)
- Functions are potentially ctors. They are when you do new Function()
    - when called with new, they get 'this' 
    - Functions automatically get property called prototype
    - arrow function don't have its prototype object
- Before 2015, this was how a class was created in JS
*/
const test_prototype = () => {
    // creating prototype object
    var prototype = {
        func: () => {
            console.log("prototype func");
        }
    }

    // Method 1: create an object with the prototype
    var Dog = Object.create(prototype);
    Dog.bark = () => {
        console.log("dog bark");
    }
    Dog.bark();
    Dog.func();

    // arrow function doesn't have the prototype
    const arrow_func = () => {
        console.log("arrow func");
    }
    // arrow_func.prototype.baz = "lol";
    // console.log('arrow func prototype baz: ', arrow_func.prototype.baz);

    // Method 2: using new ctor
    function fake_ctor(){
        this.bar = "bar";
    }
    fake_ctor.prototype.baz = "baz";
    var fake_obj = new fake_ctor();
    console.log('fake ctor prototype baz: ', fake_obj.baz);
}
// test_prototype()

const test_override = () => {
    function Vehicle(type){
        this.type = type;
    }
    Vehicle.prototype.drive = () => {
        console.log("vehicle drive");
    }
    // creating new object, which we override
    let car = Object.create(Vehicle.prototype);
    car.drive = () => {
        console.log("car drive");
    }
    car.drive();
}
// test_override();

/*
Symbols: unique object identifier from strings
Two symbols created from the same string are NOT the same
- Avoid name collision in prototype:
    - toString vs Symbol('string')
= Common symbols: iterator
Symbols are created standalone, or globally.
- Standalone: Symbol("str") 
- stored in a global symbol registry. Symbol.for(string) will get you that symbol
SO, STICK TO THE GLOBALLY STORED SYMBOL!!
*/
const test_symbol = () => {
    let arr = [1,2,3,4,5];
    console.log('array to string: ', arr.toString());
    let toStringSym = Symbol("toString")
    Array.prototype[toStringSym] = () => {
        return "Rico says: " + arr.toString();
    }
    console.log('testing symbol: ', arr[toStringSym]());
    console.log('Two symbols are not the same: ', toStringSym === Symbol("toString"));
    
    let toStringSymGlobal= Symbol.for('toString');
    let toStringSymGlobal2= Symbol.for('toString');
    console.log('Two globally stored symbols from for are the same: ', toStringSymGlobal === toStringSymGlobal2);
}
// test_symbol();

/*
- Getter & setter
    - getter cannot have parameters
- By default, class methods are public
*/
const test_getter_setter = () => {
    class Matrix {
        constructor(width, height) {
            this._width = width;
            this._height = height;
            this._data = Array.from({length: height}, 
                (v,k) => Array.from({length: width}, (v,k) => 0));
        }
        get data(){
            return this._data;
        }

        set data(new_data){
            if (new_data.length === 0 || new_data[0].length === 0){
                console.log('Matrix Creation Failed, make sure a 2D array is passed in');
                return
            }
            if (new_data.some(row => row.length !== new_data[0].length)){
                console.log("Matrix Creation Failed, 2D array should have the same dimension");
                return
            }
            this._data = new_data;
            this._height = new_data.length;
            this._width = new_data[0].length;
        }



    };
    let m = new Matrix(4,3);
    console.log("data:", m.data);
    m.data = [[1,2,3], [4,5,6]]
    console.log("after setting the new data: ", m.data);
    return Matrix;
}


/**
 * Iterator interface:
    - Get the iterator: obj[Symbol.iterator]()
        - setting the iterator Class.prototype[Symbol.iterator] to a function, not an
        arrow function
    - call next(). This is effectively what the for-of loop does
    - next() must return at least a "done"
 */
const test_iterator = () => {
    // return an iterator:
    let Matrix = test_getter_setter();
    class MatrixIterator {
        constructor(matrix){
            this._x = 0;
            this._y = 0;
            this._matrix = matrix;
        }
        // will return an "end" iterator, one position after the end of the matrix
        next(){
            if(this._x === 0 && this._y === this._matrix._height){
                return {done: true};
            }
            let value = this._matrix.data[this._y][this._x];
            this._x++;
            if (this._x === this._matrix._width){
                this._x = 0;
                this._y++;
            }
            return {value, done: false};
        }
    }
    Matrix.prototype[Symbol.iterator] = function () {
    // Be careful with the iterator prototype
    // Matrix.prototype[Symbol.iterator] = () => {
        return new MatrixIterator(this);
    }
    console.log('iterating thru Matrix: ');
    let iterable_m = new Matrix(2,3);
    iterable_m.data = [[1,2,3], [4,5,6]];
    let m_it = iterable_m[Symbol.iterator]();
    console.log(m_it);
    console.log('calling next on it: ', m_it.next());
    // iterator can be used in for-of loops
    for (let m_element of m){
        console.log(m_element);
    }
}

/**
 * Polymorphism: calling a function using super()
 *  - obj instanceOf class
 */
const test_polymorphism = () => {
    let Matrix = test_getter_setter();
    class Vector extends Matrix{
        constructor(n){
            super(n, 1);
        }
    }
    let vec = new Vector(3);
    console.log(vec);
    console.log('if is an instance of matrix: ', vec instanceof Matrix);
}
// test_polymorphism();

/*
call is a way to bind a function to an object, then call it.
- Must be function, not an arrow function
*/
const test_call = () => {
    function some_func(){
        console.log(this.a);
    }
    const o = {a: 1, b: 2};
    some_func.call(o);
}
// test_call();

/*
- Freezing an object is Object.freeze(), then you set it readonly
- Object.assign does not have deep copy. Also, each field could be overwritten
*/
const test_freeze = () => {
    const o = {a: 1, b: 2};
    Object.freeze(o);
    try{
        o.a = 3;
    } catch(e){
        console.log("Once you do Object.freeze, you can't change the object");
    }

    obj1 = {a: 1, b: 2};
    obj2 = {b:3, c:4};
    target = {d:5}
    // see {a:1, b:3, c:4, d:5}
    newTarget = Object.assign(target, a, b)
    Object.assign()
}
// test_freeze();

/**
 * Deep copy: lodash.cloneDeep() can do the trick, too
 */
const test_copy = () => {
    let o = {a: 1, b: 2};
    let o2 = {c:o};
    let shallow_copy = {...o2};
    let deep_copy = JSON.parse(JSON.stringify(o2));
    shallow_copy.c.a = 100
    console.log('shallow copy: ', shallow_copy);
    console.log('deep copy: ', deep_copy);
}
test_copy()