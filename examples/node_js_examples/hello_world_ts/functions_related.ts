/*
    - PropertyDescriptor has metadata of whether a class is writable, configurable, etc.
        - You still need tsconfig.json and have experimentalDecorators:true in there
    - target:
        - for method or accessor decorator, target is the class prototyoe
        - for class decorator, target is the constructor
*/
function dec(target: object, property_key: string, descriptor: TypedPropertyDescriptor<any>){
    var orig_method = descriptor.value;
    descriptor.value = function(...args: any[]){
        console.log(`Logging ${property_key} with args: ${args}`);
        console.log(`${target}`)
        return orig_method.apply(this, args);
    }
    return descriptor;
}

function classDec(target: Function){
    console.log(`you can see the entire code of ctor: ${target}`);
    // console.log(`decorating the ctor of class ${target.name}`);
}

@classDec
class Calc{
    constructor(){
        console.log("Constructor");
    }

    @dec
    add(a: number, b:number): number{
        return a + b;
    }
}

let c = new Calc();
c.add(1,2);

// /*
// - Using python-like class decorator 
// */
// function ReadOnly(target: any, key: string){
//     // TODO is how you get keys from an object? 
//     const descriptor = Object.getOwnPropertyDescriptor(target, key) || {}; 
//     descriptor.writable = false;
//     Object.defineProperty(target, key, descriptor);
// }

// class Foo{
//     @ReadOnly
//     bar = "bar";
// }

// const foo = new Foo();
// foo.bar = "barz";
// console.log(foo.bar);


/**
 * class
 *  - module.d.ts is a decalartion file. with class declarations only
 *  - shorthand for class
 */
class Foo2{
    // short hand for public readonly number: i; 
    constructor(public readonly i: number){}
}
let foo2 = new Foo2(200)
console.log(foo2.i);