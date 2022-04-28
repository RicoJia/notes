#!/usr/bin/python3
def test_counter(): 
    """
    1. counter is a dictionary that counts how many times an item has shown up
        - you can incorporate in another list
    2. can do +, - counters, where - is to remove the intersection of two dictionaries
    """
    from collections import Counter
    words = [ 'look', 'into', 'my', 'eyes', 'look', 'into', 'my', 'eyes', 'the', 'eyes', 'the', 'eyes', 'the', 'eyes', 'not', 'around', 'the', 'eyes', "don't", 'look', 'around', 'the', 'eyes', 'look', 'into', 'my', 'eyes', "you're", 'under']
    cnt = Counter(words)
    top_three = cnt.most_common(3)
    print(top_three)

    another_list = ["hug", "hug", "hug", "hug", "hug", "hug"]
    cnt.update(another_list)
    another_top_three = cnt.most_common(3)

    cnt_2 = Counter(another_list)
    print("combining counts: ", cnt + cnt_2)
    print("subtracting counts: ", cnt - cnt_2)
    


if __name__ == "__main__":
    test_counter()
