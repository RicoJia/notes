def test_str():
    string = "123"
    # see a123b123c
    print(string.join("abc"))
    # {} is called the placeholder 
    print('{}'.format(string)) 

test_str()
