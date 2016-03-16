library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mult16x16UnsignedSigned is
    Port(a : in unsigned(15 downto 0);
         b : in signed(15 downto 0);
         y : out signed(31 downto 0));
end mult16x16UnsignedSigned;

architecture behavioral of mult16x16UnsignedSigned is
begin
    y <= resize(resize(b, 17) * signed(resize(a, 17)), 32);
end behavioral;
