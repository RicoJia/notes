#! /usr/bin/python3 
def test_confirmation(): 
    def request_confirmation(message: str, suffix: str = " (y/n): ") -> bool:
        message += suffix
        proceed = input(message)
        if proceed.lower() == "y":
            return True
        elif proceed.lower() == "n":
            return False
    request_confirmation("do you love")

if __name__ == "__main__": 
    test_confirmation()
