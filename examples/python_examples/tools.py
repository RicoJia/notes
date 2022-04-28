#!/usr/bin/python3
def test_counter(): 
    """
    1. counter is a dictionary that counts how many times an item has shown up
    """
    from collections import Counter
    words = [ 'look', 'into', 'my', 'eyes', 'look', 'into', 'my', 'eyes', 'the', 'eyes', 'the', 'eyes', 'the', 'eyes', 'not', 'around', 'the', 'eyes', "don't", 'look', 'around', 'the', 'eyes', 'look', 'into', 'my', 'eyes', "you're", 'under']
    cnt = Counter(words)
    top_three = cnt.most_common(3)
    print(top_three)



if __name__ == "__main__":
    test_counter()
