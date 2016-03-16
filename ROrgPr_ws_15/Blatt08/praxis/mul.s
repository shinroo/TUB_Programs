	.data
x:	.word 12
y:	.word 4
r:	.word 0
one:	.word 1

	.text
	.globl main

main:	
	lw $t1,one		# t1 = 1

	lw $t7,x		# t7 = x
	lw $t8,y		# t8 = y

	add $t9,$zero,$zero	# t9 = result = 0
loop:
	beq $t8,$zero,end	# if y == 0 then end
	sub $t8,$t8,$t1		# y--
	add $t9,$t9,$t7		# result += x
	beq $zero,$zero,loop 	# jmp loop
end:
	sw $t9,r
