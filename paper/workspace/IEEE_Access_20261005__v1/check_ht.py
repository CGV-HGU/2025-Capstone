with open('main.log', 'r', encoding='latin1') as f:
    text = f.read()

idx = text.find('Optional argument of \\twocolumn too tall')
if idx != -1:
    print(text[idx-200:idx+300])
