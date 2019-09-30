from sets import Set





class Parent:

    name = "Parent"
    payload = None
    args = []

    def __init__(self, value):
        self.value = value


    @classmethod
    def instantiate(cls, args):
        return cls.payload(args)


    @classmethod
    def get_args(cls):
        return cls.args


    def write(self):
        print(self.value)



class Child(Parent):

    name = "Child"
    payload = Set
    args = ["blub", "hallo", 42]

    def __init__(self, value, greeting, number):
        super(value)
        self.greeting = greeting
        self.number = number


    def write(self):
        print(self.greeting)
        print(self.number)

    # @classmethod
    # def instantiate(cls, args):
    #     return cls.payload(args)


if __name__ == '__main__':
    classes = [Child]
    for cls in classes:
        args = cls.get_args()
        obj = cls.instantiate(args)
        print(obj)
