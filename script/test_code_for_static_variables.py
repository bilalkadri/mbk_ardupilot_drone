index=0


def test_function():
   global index
   index=index+1
   print("index:",index)


if __name__ == '__main__':
    while index<10:
        test_function()