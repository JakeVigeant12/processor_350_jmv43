def genShift(direction, numBits, numBitsString):
    print("module "+ numBitsString + "_bit_" + direction + "_shift(data, out);")
    print("input [31:0] data;")
    print("output [31:0] out;")
    for i in range (32):
        iPrint = str(i)
        sha = str(i+numBits)
        if(direction == "right"):
            print("assign out["+iPrint+"] = data["+sha+"];")
        if(direction == "left"):
            print("assign out["+sha+"] = data["+iPrint+"];")
    print("endmodule")


genShift("right",16,"sixteen")

for i in range(32):
    print(" addRes["+str(i)+"],",  end="" )