main:
    addi    t0, x0, 4
    lw      s1, 0(t0)
    addi    t0, x0, 8
    lw      s0, 0(t0)
    addi    t1, s1, -2
    blt     t1, x0, end_main
    beq     t1, x0, end_main
    addi    t1, s1, -1
    add     t1, t1, t1
    add     t1, t1, t1
    add     s2, s0, t1
    addi    t0, s0, 4

loop_main:
    beq     t0, s2, end_main
    add     a0, t0, x0
    jal     ra, median_of_three
    sw      a0, 0(t0)
    addi    t0, t0, 4
    jal     x0, loop_main

end_main:
    jal     x0, end_main

median_of_three:
    lw      t0, -4(a0)
    lw      t1, 0(a0)
    lw      t2, 4(a0)
    blt     t1, t0, m_swap01
m_after01:
    blt     t2, t1, m_swap12
m_after12:
    blt     t1, t0, m_swap01_2
m_after01_2:
    add     a0, t1, x0
    jalr    x0, 0(ra)

m_swap01:
    add     t3, t0, x0
    add     t0, t1, x0
    add     t1, t3, x0
    jal     x0, m_after01

m_swap12:
    add     t3, t1, x0
    add     t1, t2, x0
    add     t2, t3, x0
    jal     x0, m_after12

m_swap01_2:
    add     t3, t0, x0
    add     t0, t1, x0
    add     t1, t3, x0
    jal     x0, m_after01_2