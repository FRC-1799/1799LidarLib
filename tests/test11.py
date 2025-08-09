class test:
    def __init__(self):
        self.val="hi"

    def printhi(self):
        print(self.val)


func = test.printhi
inst = test()

func(inst)