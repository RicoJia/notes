/*
- Asynchrony
- Callback only coding suffers from Inversion of Control? (IoC), lack of linear reasonability
- Synchronous is not multi-threaded: two threads can be blocked by I/O (like networking)
    - Async is two threads can continue while being blocked.
    - So, synchronous + multi-threaded is great for CPU-intensive and I/O intensive work
- Ways to achieve asynchrony:
    - Callbacks: very common, but sometimes you just need to retunr a
*/
setTimeout(()=>{console.log("setTimeout");}, 500);

// 1. Create a promise that immediately resolves to 15. then set a callback
// fiftenn_promise.then actualy returns another promise
fifteen_promise = Promise.resolve(15);
fifteen_promise.then(value => console.log(value));
console.log(typeof fifteen_promise);

// 2. Sample usage
function simulateFetchingWebpage(){
    // when creating a promise, you get resolve and reject, which marks the promise's status as "fulfilled" or "rejected"
    return new Promise((resolve, reject) => {
        setTimeout(() => {
            const data = "hola";
            console.log(data);
            resolve(data);
        }, 200);
    });
}
// use the promise
simulateFetchingWebpage();
// use promise again, but with another callback. You SHOULD catch errors thrown too.
simulateFetchingWebpage().then(
    // data comes from the resolve previosly
    data => {
        console.log("Can reuse the promise", data);
    }
). catch(
    error => console.log("error could be thrown, by reject", error)
)
    

// 3. Reject
function testReject(){
    return new Promise(
        (_, reject) => reject(new Error("Fail"))
    ).catch(
        reason => {
                // here you will see the full tracestack
                console.log("Caught failure ", reason);
            return "nothing";
        }
    );
}
testReject();


// // Async function returns a promise, implicitly
// // If inside it, you return a value, that's resolved. If you throw an error, that's rejection
// then, javascript will continue to other codes (like "haha")
// if inside the async function, we need to await
async function fetch_url(url){
    try{
        // pause the function until to wait for a promise to be resolved
        // await returns a value resolved from promise
        let response = await fetch("http://0.0.0.0:8000/");
        let data = await response.text();
        console.log("fetched html, ", data);
    }catch(e){
        console.log("an error occurred: " + e.message);
    }
}

fetch_url();
console.log("hahahah");