#            -------------------RADIXSORT--------------------
#
# Vorgabe von Daniel Stelzer, TU Berlin
#
# Ihre Aufgabe:
# Radixsort implementieren, die Matr. Nummern sind im Array einzutragen.
# radix bekommt als Eingabe das Array und die feste Länge (nicht ändern) des Arrays.
# Das sortierte Array ist auf dem Stack zu speichern (Auf die richtige Reihenfolge achten).
# Der Rücksprung aus radix in die main ist bereits in dieser Vorgabe enthalten.

	.data
array: .word 369264, 375977, 372928, 371606 #Martnummer hier durch Komma getrennt
#array: .word 462963, 779573, 829273, 606173

bucket: .space 160
# bucket contains enough space for 10 * 4 ints, 10 digits in decimal, 4 ints to sort
# every 16 bytes represent a new digit, every 4 bytes in a 16 byte group are the
# available space for an int (max 4 for one group (digit))
vacant: .space 40
# vacant keeps track of number of stored ints in a digit group in bucket
# 10 digits * 4 byts = 40 bytes

#----------NICHT aendern-------------
n:     .word 4
text1: .asciiz "Willkommen zur Programmierhausaufgabe RORG WS15/16.\nIhr Radixsort hat folgende Reihenfolge sortiert.\n"
newline: .asciiz "\n"

#
# main
#
	.text
	.globl main

radix:

###
# Insert your code here
###


	# v0 --> large
	# a0 --> array
	# a1 --> size

	# t0 --> i

# load bucket and vacant addresses
la  $s0, bucket
la  $s1, vacant

# 'poison' bucket array by intialising with -1
addi $t0, $zero, 0    # i = 0
addi $t1, $zero, 160  # loop limit
addi  $t3, $zero, -1  # value to be stored
forPoison:
          # for ( i = 0, i < 160 , i+4)
          bge   $t0, $t1, end_forPoison
                add   $t2, $t0, $s0 # move pointer
                sw    $t3, 0($t2)   # store -1
          addi  $t0, $t0, 4 # i + 4
          j forPoison
end_forPoison:

# initialise vacant array with 0
addi  $t0, $zero, 0   # i = 0
addi  $t1, $zero, 10  # loop limit
forVacant:
          # for ( i = 0 ; i < 10 ; i++ )
          bge   $t0, $t1, end_forVacant
                sll   $t2, $t0, 2     # $t2 = i * 4
                add   $t2, $s1, $t2   # $t2 = &vacant[i]
                sw    $zero, 0($t2)   # vacant[i] = 0
          addi  $t0, $t0, 1 # i++
          j     forVacant
end_forVacant:

addi $t0, $zero, -1	# $t0 is outer loop index, sum
addi $s3, $zero, 10 # divisor
addi $s2, $zero, 10 # multiplicand to multiply divisor by ten

sort:
beq $t0, $zero, end_radix   # if sum is zero, sort complete
    add $t0, $zero, $zero   # sum is zero at start of loop
    add $s6, $zero, $zero   # index for array, reset every time
    addi $s5, $zero, 4      # escape condition

bucket_fill:
    beq $s6, $s5, bucket_filled
    sll $t4, $s6, 2 # multiply by four to get address
    add $t4, $t4, $a0 # get correct address in array
    lw $t7, 0($t4) # load value of array[i] into t7

    div $t7, $s3 # divide number by divisor
    mflo $t4 # quotient
    mfhi $t5 # remainder

    add $t0, $t0, $t4 # sum = sum + quotient

    div $s3, $s2 # get correct divisor for remainder
    mflo $t6

    div $t5, $t6 # get correct digit from remainder
    mflo $t5

    # get to correct place in bucket
    # 1. Get correct bucket: rem*16
    # 2. Get correct index in bucket: (vacant)+rem*4

	sll $t6, $t5, 2 # $t6 - offset for vacant
    sll $t5, $t5, 4 # $t5 - offset for correct bucket from bucket array

    add $s4, $t6, $s1 # get the correct address for vacant offset
    lw $t2, 0($s4) # load correct vacant offset as int in $t2

    sll $t1, $t2, 2 # get the correct address offset in vacant

    add $t1, $t1, $t5 # final address offset for the bucket
    add $t1, $t1, $s0 # add offset to bucket address
    sw $t7, 0($t1) # finally store word in correct bucket position

    addi $t2, $t2, 1 # increment vacant index
    sw $t2, 0($s4)   # store new vacant index
    addi $s6, $s6, 1 # increment vacant index
    j bucket_fill
bucket_filled:

add 	$t4, $zero, $zero # i = 0
addi 	$t5, $zero, 160  # loop limit
addi 	$s5, $zero, -1 # poison checker
add 	$s6, $zero, $zero # array index

bucket_empty:
          bge   $t4, $t5, bucket_emptied # if index $t4 = 160, array resorted, bucket empty
                add   $t2, $t4, $s0 # move pointer
                lw 		$t7, 0($t2) # get value from bucket
                beq 	$t7, $s5, no_add # if value is -1 (poisoned), don't store
                sll 	$t6, $s6, 2 # shift index to get byte address offset
                add 	$t6, $t6, $a0 # add offset to array address
                sw  	$t7, 0($t6)   # store in array
                addi 	$s6, $s6, 1   # increment array index if stored
          no_add:
          addi  $t4, $t4, 4 # i + 4   # +4 to index, next position in bucket
    j bucket_empty
bucket_emptied:
# repoison bucket once sorted array is stored
addi 	$t4, $zero, 0    # i = 0
addi 	$t5, $zero, 160  # loop limit
addi  $s5, $zero, -1  # value to be stored

repoison_bucket:
          # for ( i = 0, i < 160 , i+4)
          bge   $t4, $t5, end_repoison
                add   $t2, $t4, $s0 # move pointer
                sw    $s5, 0($t2)   # store -1
          addi  $t4, $t4, 4 # i + 4
          j repoison_bucket
end_repoison:

# reset vacant values to 0
addi  $t4, $zero, 0   # i = 0
addi  $t5, $zero, 10  # loop limit

revacant:
          # for ( i = 0 ; i < 10 ; i++ )
          bge   $t4, $t5, end_revacant
                sll   $t2, $t4, 2     # $t2 = i * 4
                add   $t2, $t2, $s1   # $t2 = &vacant[i]
                sw    $zero, 0($t2)   # vacant[i] = 0
          addi  $t4, $t4, 1 # i++
          j     revacant
end_revacant:

mult $s3, $s2 # multiply divisor by ten to get next digit in numbers
mflo $s3 # store new divisor
j sort

end_radix: # push to stack

addi $t0, $zero, 4 # set index to 4 (four elements)

# store sorted array on stack
# Store in ascending order
stack_me_up:
    # for (i = 0; i < n; i++)
    beq $t0, $zero, fully_stacked  # if index = 0, array stored in stack
    addi $t0, $t0, -1 # decrementing index 
    addi $sp, $sp, -4 # move stack pointer
    sll $t1, $t0, 2   # index = index*4, to get byte address
    add $t1, $t1, $a0 # add offset to array address
    lw $t1, 0($t1)    # load word from array
    sw $t1, 0($sp)    # store word in stack
    j stack_me_up     # loop back
fully_stacked:

#return to main
jr 	$ra


main:

# Tttle
	la 		$a0, text1
	li 		$v0, 4
	syscall
#end title

	addi	$sp, $sp, -4		# save return adress
	sw		$ra, 0($sp)

	la		$a0, array		# array adress
	lw		$a1, n

	jal	radix

# print 1
	lw 		$a0, 0($sp)
	addi 	$sp,$sp,4
	li		$v0, 1
	syscall
	la 		$a0, newline
	li 		$v0,4
	syscall

# print 2
	lw 		$a0, 0($sp)
	addi 	$sp,$sp,4
	li		$v0, 1
	syscall
	la 		$a0, newline
	li 		$v0,4
	syscall
# print 3
	lw 		$a0, 0($sp)
	addi 	$sp,$sp,4
	li		$v0, 1
	syscall
	la 		$a0, newline
	li 		$v0,4
	syscall
# print 4
	lw 		$a0, 0($sp)
	addi 	$sp,$sp,4
	li		$v0, 1
	syscall
	la 		$a0, newline
	li 		$v0,4
	syscall

	lw		$ra, 0($sp)
	addi	$sp, $sp, 4
	jr		$ra

#
# end main
#
