library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity add33 is
    Port(a : in signed(31 downto 0);
         b : in signed(31 downto 0);
         y : out signed(32 downto 0));
end add33;

architecture behavioral of add33 is
signal a31, b31 : std_logic;
begin
    a31 <= a(31);
    b31 <= b(31);
    y <= (a(31)&a)+(b(31)&b);
end behavioral;
