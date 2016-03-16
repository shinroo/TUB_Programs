        .data
# without absolute path, files aren't recognized
file_in:   .asciiz "/home/filaret/Uni/RechOrg/assembler/MyVowelCount.txt"	# filename for input
#file_in:   .asciiz "/home/filaret/Uni/RechOrg/assembler/LoremIpsum.txt"

file_out:  .asciiz "/home/filaret/Uni/RechOrg/assembler/VokaleMyVowelCount.txt"		# filename for output
#file_out:  .asciiz "/home/filaret/Uni/RechOrg/assembler/VokaleLoremIpsum.txt"

after_letter: .asciiz ": "
next_line: .asciiz "\n"

vokale:    .word 'A', 'E', 'I', 'O', 'U',
array:     .space 512 # reserves 128 * 4 bytes for an int array
buffer:    .space 1
number: .word w1, w2, w3, w4 # array for the counter numbers, each digit will be used
	.text

main:
  ###############################################################
  # Set every element of int array[128] to 0
  add   $t0, $zero, $zero      # index $t0 = 0
  la    $s1, array      # put address of array into $s1

  for1:
    slti $t1, $t0, 128		# do while $t0 < 128
    beq  $t1, $0, end_for1	# if $t0 >= 128, goto end_for1
    sll  $t2, $t0, 2		# $t2 = 4*$t0
    add  $t2, $s1, $t0		# $t2 = address of array[$t0]
    sw   $0, 0($t2)     	# set array[$t2] to 0
    addi $t0, $t0, 4      	# $t0++
    j    for1             	# jump to for1
  end_for1:
  ###############################################################
  # Open file for reading
  li    $v0, 13			# system call for open file
  la    $a0, file_in     	# input file name
  li    $a1, 0			# Open for reading
  li    $a2, 0			# mode is ignored
  syscall			# open a file (file descriptor returned in $v0)
  move  $s6, $v0		# save the file descriptor
  ###############################################################

  add  $t0, $0, $0		# index $t0 = 0
  addi $t6, $0, 1024
    # read the first $t6 characters or end when EOF
    # each letter has its equivalent in ASCII, so when it appears, its index in array is increased by one
  for2:
    slt  $t1, $t0, $t6		# do while $t0 < $t6 - read the first $t6 characters
    beq   $t1, $0, end_for2

    li    $v0, 14		# system call for read from file
    move  $a0, $s6		# $a0 = previously saved filedescriptor $s6
    la    $a1, buffer		# address of buffer to which we store the character
    li    $a2, 1		# number of characters to read
    syscall			# read a character from a file to buffer
    
    add  $t3, $0, $0		# if $v0 < 1, EOF or error - goto end_for2
    slt  $t3, $t3, $v0		#
    beq  $t3, $0, end_for2	#

    lw    $t1, 0($a1)      	# set $t2 to $a1[0] (to buffer[0])
    
    sll   $t1, $t1, 2		# $t1 = 4*$t1
    add   $t2, $t1, $s1		# get the address of array[$t1] - array[char_just_read]
    lw    $t1, ($t2)		# get the value of array[$t2]
    addi  $t1, $t1, 1		# increase $t1 by one
    sw    $t1, ($t2)		# set array of index $t2 to $t1 (increased by one)

    addi  $t0, $t0, 1		# $t0++
    
    j for2
  end_for2:
  ###############################################################
  # Close the file
  li    $v0, 16		# system call for close file
  move  $a0, $s6	# file descriptor to close
  syscall		# close file
  ###############################################################
  # open to write
  li $v0, 13 # syscall for open file
  la $a0, file_out # specify file
  li $a1, 1 # open for writing
  li $a2, 0 # ignore mode
  syscall
  move $a0, $v0 # store file descriptor in $a0
  move $s6, $v0 # store file descriptor in $s6, for later use

  ###############################################################

  add $t0, $0, $0	# index $t0 = 0
  # read the vowel count array
  la   $s1, vokale	# load address of array vokale
  la   $s2, array	# load address of array array
  la   $s3, number	# load address of array number
  for3:
  
    slti  $t3, $t0, 5   # do while $t0 < 5
    beq   $t3, $0, end_for3

    sll $t3, $t0, 2	# $t3 = 4*$t0
    add $t3, $t3, $s1	# $t3 = address of vokale[$t0]

    la $a1, 0($t3)	# print the letter {A, E, I or O} to file
	li $a2, 1	# print one character
    li $v0, 15	# syscall for print
    syscall	

    la $a1, after_letter # print ": "
    li $a2, 2	# print two characters
    li $v0, 15	# syscall for print
    syscall
    
    lw $t3, 0($t3)	# $t3 =  value of vokale[$t0]
    sll $t3, $t3, 2	# $t3 = $t3*4
    add $t3, $t3, $s2	# $t3 = address of array[$t3]
    
    lw $s7, 0($t3)	# $s7 =  value of array[$t0], (word count) , capital
    
    lw $t3, 128($t3)	# $t3 = array[small_letter]
    
    add $s7, $s7, $t3	# $s7 = number of vowels (capital + small)
	j itoa

cont_for3:


    la $a1, next_line	# print to stdout "\n"
    li $a2, 1		# print 1 character
    li $v0, 15		# syscall for print
    syscall		#
    
    addi $t0, $t0, 1	# $t0 ++
    j for3

# Integer to Ascii
# An int is divided by ten, until the quotient is equal to zero
# After each division, the remainder is the next digit in the number,
# from first to last
# Thus, we can parse the number and convert each digit to an ascii symbol
# To each digit, 48 is added to get the ascii decimal code for the digit
# The ascii representation is stored in the array number, but in reverse:
# 1024 becomes 4201
# Afterwards, the number is printed backward, to display it in its correct order

itoa:
	addi $t7, $zero, 10	# base 10 division
	move $t6, $s7		# $t6 is integer to be converted and first dividend
	add $t4, $0, $0		# $t4 is an index, will be used in print

loop:
	div $t6, $t7		# divide number by 10, get quot and remainder
	mflo $t6			# t6 is quotient
	mfhi $t5			# t5 is remained, the digit to be converted
	
	sll $t1, $t4, 2		# $t1=$t4*4, $t1 used for number address
	add $t1, $t1, $s3	# $t1 is the correct index address of number
	
	addi $t5, $t5, 48	# convert digit to ascii code, decimal
	sw $t5, 0($t1)		# the ascii code is stored in the number array

	addi $t4, $t4, 1	# increment index
	beq $t6, 0, print_loop # if quotient is zero, start printing, all digits parsed
	j loop				# otherwise, continue with next digit
print_loop:
	addi $t4, $t4, -1	# index adjusted to point to correct digit

	sll $t1, $t4, 2		# $t1 = $t4*4, $t1 is used for number address
	add $t1, $t1, $s3	# $t1 is the correct index address of number

	la $a1, 0($t1)		# load ascii digit in buffer for print
	li $a2, 1			# print a single character
	li $v0, 15			# syscall for print
	syscall

	beq $t4, $0, cont_for3 # once all digits are printed, continue
	j print_loop		# otherwise, continute with next digit

  end_for3:
  ###############################################################
  move $a0, $s6		# address of output file
  li $v0, 16		# syscall for close file
  syscall
  li $v0, 10		# syscall for exit program
  syscall
