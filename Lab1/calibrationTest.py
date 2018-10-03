keylistLeft = []
keylistRight = []
lPwmTranslation = {}
rPwmTranslation = {}

def readCalibratedSpeeds():
    global lPwmTranslation, rPwmTranslation, keylistLeft, keylistRight

    l = open("LeftSpeedCalibration.txt", "r")
    for line in l:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        lPwmTranslation[rpsValue] = pwmValue
    l.close()

    r = open("RightSpeedCalibration.txt", "r")
    for line in r:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        rPwmTranslation[rpsValue] = pwmValue
    r.close()

    keylistLeft = list(lPwmTranslation.keys())
    keylistLeft.sort()
    keylistRight = list(rPwmTranslation.keys())
    keylistRight.sort()

def findPwmValues(rpsLeft, rpsRight):
    if rpsLeft in keylistLeft:
        pwmLeft = lPwmTranslation[rpsLeft]
    elif rpsLeft < min(keylistLeft):
        pwmLeft = lPwmTranslation[keylistLeft[0]]
    else:
        nextElem = 1
        for elem in keylistLeft:
            if elem == max(keylistLeft):
                pwmLeft = lPwmTranslation[elem]
                break
            elif rpsLeft > elem:
                if rpsLeft < keylistLeft[nextElem]:
                    pwmLeft = lPwmTranslation[keylistLeft[nextElem]]
                    break
            nextElem += 1

    if rpsRight in keylistRight:
        pwmRight = rPwmTranslation[rpsRight]
    elif rpsRight < min(keylistRight):
        pwmRight = rPwmTranslation[keylistRight[0]]
    else:
        nextElem = 1
        for elem in keylistRight:
            if elem == max(keylistRight):
                pwmRight = rPwmTranslation[elem]
                break
            elif rpsRight > elem:
                if rpsRight < keylistRight[nextElem]:
                    pwmRight = rPwmTranslation[keylistRight[nextElem]]
                    break
            nextElem += 1

    return (pwmLeft, pwmRight)


readCalibratedSpeeds()

print(lPwmTranslation)
print(rPwmTranslation)

test = findPwmValues(0.5, 0.5)
print(test)
