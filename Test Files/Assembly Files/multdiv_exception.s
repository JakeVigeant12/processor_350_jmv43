nop             # Values initialized using addi (positive only)
nop             # Author: Oliver Rodas
nop
nop             # Multdiv without Bypassing
nop 			# Multdiv Tests
addi $r1, $r0, 32767    # r1 = 32767
sll $r1, $r1, 16        # r1 = 2147418112
addi $r1, $r1, 65535    # r1 = 2147483647 (Max positive integer)
nop
nop
mul $19, $r1, $r1	# overflow: rstatus = 4