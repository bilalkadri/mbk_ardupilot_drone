
global pub_image
pub_image=7
print(pub_image)

def another_function():
    global pub_image
    print(pub_image)

def image_publisher():
    global pub_image
    pub_image=2
    print(pub_image)


if __name__ == '__main__':

        image_publisher()
        pub_image=3
        another_function()

   