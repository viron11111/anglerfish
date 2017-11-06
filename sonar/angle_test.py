import cmath

#timestamps
tab = 113.0
tac = 62.5
tad = 62.5

#absolute distances
dab = 173.2
dac = 173.2
dad = 100

#speed of sound underwater
c = 1.484

#angles
aab = 0
aac = -1.0472
aad = -0.523598776


theta_ab_neg = aab-cmath.acos((c*tab)/dab)
theta_ab_pos = aab+cmath.acos((c*tab)/dab)

theta_ac_neg = aac-cmath.acos((c*tac)/dac)
theta_ac_pos = aac+cmath.acos((c*tac)/dac)

theta_ad_neg = aad-cmath.acos((c*tad)/dad)
theta_ad_pos = aad+cmath.acos((c*tad)/dad)

theta_ab = [theta_ab_pos,theta_ab_neg]
theta_ac = [theta_ac_pos,theta_ac_neg]
theta_ad = [theta_ad_pos,theta_ad_neg]

print "theta_ab: ", theta_ab
print "theta_ac: ", theta_ac
print "theta_ad: ", theta_ad