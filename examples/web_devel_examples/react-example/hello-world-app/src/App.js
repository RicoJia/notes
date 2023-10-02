// export default App;
import React, { useState, useEffect } from 'react';
import './App.css';

/**
 * - useState: react hook to initialize variable isVisible to true
 *  - function setIsVisible to update isVisible. Note setIsVisible has not been defined yet
 * - useEffect: react hook to run side effects, like fetching data, etc.
 *  - effectively, mounting the component, update it, and unmount it.
 * - setInterval() returns an ID of the interval.
 * - setIsVisible() is defined to be prev => !prev. The passed in value is the registered 
 *  - value isVisible. The result will go back to isVisible, too. 
 *  - This is a state setter
 */
function App() {
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    const interval = setInterval(() => {
      setIsVisible(prev => !prev);
    }, 2000); // 500ms for 2 toggles = 1Hz

    /*
    - this effectively registers setInterval in the JS event loop. clearInterval is built-in JS
    function, prevent leak
    */
    return () => clearInterval(interval); // Cleanup interval on component unmount
  }, []);

  return (
    <div className="App">
      <div className="rectangle-container">
        {Array(5).fill(null).map((_, idx) => (
          <div
            key={idx}
            className={`rectangle ${isVisible ? 'visible' : 'hidden'}`}
          ></div>
        ))}
      </div>
    </div>
  );
}

export default App;