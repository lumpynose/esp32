class Password:
    def __init__(self, file = "passwd.txt"):
        self.file = file

    def fetch(self):
        with open(self.file, 'r') as reader:
            input = reader.readline(-1)
            passwd = input.rstrip('\n')

        return passwd
