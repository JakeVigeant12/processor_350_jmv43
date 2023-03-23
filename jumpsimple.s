nop
nop
nop
addi    $r1, $r0, 4     # $r1 = 4
addi    $r2, $r0, 5     # $r2 = 5
j test
addi $r3, $r0, 7        #r3 should NOT have 7
addi $r3, $r3, 7        #r3 should NOT have 14
addi $r3, $r3, 7        #r3 should NOT have 14

test:
addi $r8, $r0, 69
