import math
KZ = -273.15
class T:
    def __init__(self,pullup,inline): self.pullup=pullup; self.inline=inline; self.c1=self.c2=self.c3=0.
    def coef(self,t1,r1,t2,r2,t3,r3):
        i1,i2,i3=1/(t1-KZ),1/(t2-KZ),1/(t3-KZ)
        l1,l2,l3=math.log(r1),math.log(r2),math.log(r3)
        L1,L2,L3=l1**3,l2**3,l3**3
        i12,i13=i1-i2,i1-i3; l12,l13=l1-l2,l1-l3; m12,m13=L1-L2,L1-L3
        self.c3=((i12-i13*l12/l13)/(m12-m13*l12/l13))
        self.c2=(i12-self.c3*m12)/l12
        self.c1=i1-self.c2*l1-self.c3*L1
    def temp(self,adc):
        adc=max(.00001,min(.99999,adc)); r=self.pullup*adc/(1-adc)
        if r-self.inline<=0: return float('inf')
        ln=math.log(r-self.inline); it=self.c1+self.c2*ln+self.c3*ln**3
        return 1/it+KZ
    def adc(self,temp):
        it=1/(temp-KZ)
        y=(self.c1-it)/(2*self.c3); x=math.sqrt((self.c2/(3*self.c3))**3+y**2)
        ln=(x-y)**(1/3)-(x+y)**(1/3); r=math.exp(ln)+self.inline
        return r/(self.pullup+r)
    def Rintr(self,temp):  # intrinsic thermistor R at temp (inline-independent SH inverse)
        it=1/(temp-KZ)
        y=(self.c1-it)/(2*self.c3); x=math.sqrt((self.c2/(3*self.c3))**3+y**2)
        ln=(x-y)**(1/3)-(x+y)**(1/3); return math.exp(ln)

th=T(430,0); th.coef(25,100000,50,31230,125,2066)

# intrinsic resistances
for tt in (25,33.3,50,125,150,215,250,290):
    print(f"  R_intrinsic({tt:>5}C) = {th.Rintr(tt):>10.1f} ohm")

# hardware anchors (under current 430,0 model)
R33 = th.Rintr(33.3)         # cold: hw reports 33.3 -> computed Rth=R33 under (430,0)
a_c = R33/430.0              # adc/(1-adc) at cold
R150 = th.Rintr(150)         # hot: (430,0) reads 150 correctly -> a_h=R150/430
a_h  = R150/430.0
R25  = th.Rintr(25)
print(f"\n  cold hw: reports 33.3C, true=25C  -> a_c={a_c:.4f}  (want computed R={R25:.0f})")
print(f"  hot  hw: reports 150 = true 150C   -> a_h={a_h:.4f}  (want computed R={R150:.0f})")

# Solve 2-point fit to (25C, 150C):  pullup*a - inline = Rtarget
# pullup*a_c - inline = R25 ; pullup*a_h - inline = R150
pullup=(R25-R150)/(a_c-a_h); inline=pullup*a_c-R25
print(f"\n=== FIT to (25C,150C): pullup={pullup:.1f}  inline_resistor={inline:.1f} ===")
print(f"   inline negative? {'YES -> NOT REPRESENTABLE (minval=0)' if inline<0 else 'no'}")
m=T(pullup,inline); m.c1,m.c2,m.c3=th.c1,th.c2,th.c3
# ceiling: model invalid when pullup*adc/(1-adc) <= inline. Find temp where computed R = inline
# i.e. R_intrinsic(T_ceiling) ... computed Rth=0 at the ceiling. As temp rises computed Rth->0.
# computed Rth(true T) = pullup*a(T) - inline where a(T)=R_intrinsic(T)/430 (hw transfer stays 430,0)
print("   model output vs TRUE temp (hardware transfer fixed at 430,0):")
for tt in (25,150,215,250,290):
    Rtt=th.Rintr(tt); a=Rtt/430.0; comp=pullup*a-inline
    rep = (1/(th.c1+th.c2*math.log(comp)+th.c3*math.log(comp)**3)+KZ) if comp>0 else float('inf')
    print(f"     true {tt:>5}C -> computed Rth={comp:>9.1f} -> reports {rep if comp>0 else 'INVALID/CEILING':>8}")

print("\n############ DISTORTION ANALYSIS ############")
print("Question: if the hardware is currently ACCURATE at print temps (working prints",
      "suggest (430,0) is ~right there), what do the inline fits read across 215-290C?\n")
# Treat (430,0) as ground truth -> hardware adc(T) = calc_adc(T;430,0)
truth=T(430,0); truth.c1,truth.c2,truth.c3=th.c1,th.c2,th.c3
def fit(Tcold,Thot):
    # anchors: cold hw reports 33.3 (true Tcold); hot hw accurate (true Thot under 430,0)
    a_c = th.Rintr(33.3)/430.0
    a_h = th.Rintr(Thot)/430.0
    Rc, Rh = th.Rintr(Tcold), th.Rintr(Thot)
    p=(Rc-Rh)/(a_c-a_h); i=p*a_c-Rc
    return p,i
for (lbl,Tc,Thot) in [("fit(25,150)",25,150),("fit(25,250)",25,250)]:
    p,i=fit(Tc,Thot)
    m=T(p,i); m.c1,m.c2,m.c3=th.c1,th.c2,th.c3
    print(f"{lbl}: pullup={p:.1f} inline={i:.1f}")
    for tt in (25,150,215,250,290):
        adc_hw=truth.adc(tt)          # what hardware really outputs (truth=430,0)
        rep=m.temp(adc_hw)
        flag="  <-- anchor" if abs(tt-Tc)<1 or abs(tt-Thot)<1 else ""
        d=rep-tt if rep!=float('inf') else 0
        print(f"   true {tt:>4}C -> reads {('CEIL' if rep==float('inf') else f'{rep:7.1f}C')}  (err {d:+6.1f}){flag}")
    print()
