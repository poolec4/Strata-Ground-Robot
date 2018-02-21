from tools import is_number

a = "123.45"
af = float(a)
print(is_number(a))
print(af)
b = "123."
bf = float(b)
print(is_number(b))
print(bf)
c = "123"
cf = float(c)
print(is_number(c))
print(cf)
d = '-0.1'
df = float(d)
print(is_number(d))
print(df)
e = '89.0\x009'
print(is_number(e))
