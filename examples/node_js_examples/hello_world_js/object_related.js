"use strict";
/*
Objects basics: 
    - using {}. So {} has 2 meanings (block scope, and object)
    - if a name is not valid for binding, double quote them. Else no need
    - Way to get all keys: Object.keys(obj)
    - Each object, and array inside objects are stored as a reference.
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
test_call();

/*
Freezing an object is Object.freeze(), then you set it readonly
*/
const test_freeze = () => {
    const o = {a: 1, b: 2};
    Object.freeze(o);
    try{
        o.a = 3;
    } catch(e){
        console.log("Once you do Object.freeze, you can't change the object");
    }
}
test_freeze();

/**
 * ================ Array ================
 * array is an subtype of object, so you can do
 */
const test_arr_basics = () => {
    var arr = ["celos", "juego", "destino"];
    typeof arr;
    // creates a new array
    let new_arr = arr.map((i) => i+"rico");
    console.log("test array basics:", new_arr);

    // splice and remove IN PLACE
    // replace 2 elements start at index 1 with "haha"
    arr.splice(1,2,"haha")
    // guess what this does ;)
    arr.splice(1,2)
    console.log("What does array look like?", arr)

    arr.push("haha");
    arr.pop()
    console.log("Arrary is a stack. push and pop are LIFO: ", arr);

}
// test_arr_basics();

const test_arr_advanced_operations = () => {
    var arr = ["celos", "juego", "destino", "destino"];
    console.log('first index for celos: ', arr.indexOf("celos"));
    console.log('last index for destino: ', arr.lastIndexOf("destino"));
    console.log('array includes destino: ', arr.includes("destino"));
    console.log('slice [0,1): ', arr.slice(0,1));
    arr.unshift("nuevo");
    console.log('unshift is inserting an element at the index', arr);
    arr.shift();
    console.log('shift is removing the beginning element: ', arr);
    console.log('array concatenation using +, You get a STRING: ', typeof(arr + ["new", "element"]));
    console.log('array concatenation using ...: ', [...arr, ...["new", "element"]]);
}
// test_arr_advanced_operations();

const test_filter = () => {
    let arr = [true, false, true, true];
    let new_arr = arr.filter(x => x);
    console.log("test filter, removed falsy elements with filter: ", new_arr);
}
// test_filter();

const test_unpacking = () => {
    let arr = [a,b,c] = [1,2,3];
    console.log('unpacking an array: ', a, b, c);
    let obj = {name: 'foo', age: 99};
    let {name} = obj;
    console.log('unpacking an object if you have the field: ', name);

    let  foo = (a,b,c) => {
        console.log("unpacking an args in an arrow function, using ...: ", a, b, c);
    }
    // ... is aka the rest/spreading operator
    foo(...arr);
}
// test_unpacking();

const test_range = () => {
    // In js there's no built in range
    let start = 20, end = 0, step = -1; 
    let range_20_0 = Array.from({length: Math.abs(end - start)}, (_, i) => start + i * step);
    console.log('range: ', range_20_0);
    console.log('array reverse: ', range_20_0.reverse());
}
// test_range();

const test_some_every = () => {
    const arr = [1, 2, 3, 3];
    console.log('if some of array is >2: ', arr.some(i => i>2));
    console.log('if every of array is >2: ', arr.every(i => i>2));
}
// test_some_every();
// ================ String ================
const test_string = () => {
    let str = "hello world";
    str.property_wont_stay = "propery_wont stay";
    console.log('property wont stay: ', str.property_wont_stay);
    console.log('string length: ', str.length);

    const originalString = "   Hello, world!   ";
    console.log("removed extraneous spacing using trim(): ", originalString.trim());
    console.log('slicing [0,2): ', str.slice(0, 2));
    console.log('padding w/ spacing: ', str.padStart(str.length + 5, "paddddd"));
}
// test_string();

/**
 * - In Javascript we can't replace the string index directly, like
    Also, we can't do name_i[1:-1]
    name_i[0]=name_i[0].toUpperCase();
    - Also, strings are encoded in 2 bytes (UTF-16). Not great idea, cuz you can get half char,
     but becoming more common with emoji
 */
const test_string_replace = () => {
    // name is a host object's name, so otherwise it will overwrite
    var name_i = "hello"
    name_i = `${name_i[0].toUpperCase()}${name_i.slice(1, name_i.length)}`;
    name_i = name_i.charAt(0).toUpperCase() + name_i.slice(1);
    console.log(name_i.slice(0,3));
}

// ================ JSON ================
const test_json = () => {
    let obj = {1:2, 3:4}
    let json_str = JSON.stringify(obj);
    console.log("json string: ", json_str);
    console.log('json object: ', JSON.parse(json_str));
}
// test_json();


// ================ Maps ================
/**
 * Regular objects store keys as strings. Maps can store things as non-keys
 * But you must use Map.has, Map.set, Map.get
 */
const test_maps = () => {
    let m = new Map();
    m[1] = "one";
    // you are setting 1 as '1' in the object, NOT in the map
    console.log('map: ', m, " if map has 1 as key", m.has(1));
    m.set(2, "two");
    console.log('map: ', m, " if map has 2 as key", m.has(2), "value is: ", m.get(2));
}
// test_maps();