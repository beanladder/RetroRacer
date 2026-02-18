# 🎯 Enhanced AI Visualization Guide

## What You'll See Above Each AI Car

The AI visualization now shows **5 lines of information** above each car in the Scene view:

```
🏁 RACING                          ← Line 1: Current Action (colored, large)
→ Will brake for sharp corner      ← Line 2: Next Intention (white)
Corner detected (45% sharp)        ← Line 3: Decision Reason (yellow)
P3 | Aggressive | Nitro:75% | RB:1.1x  ← Line 4: Internal State (gray)
142 km/h                           ← Line 5: Speed (white, bold)
```

---

## Line 1: Current Action (What AI is Doing NOW)

Shows the immediate action with color coding and emoji:

### Racing States:
- **🏁 RACING** (Green) - Following racing line normally
- **⏸ WAITING** (Gray) - Before race starts
- **🐌 SLOW** (White) - Moving slowly

### Aggressive Actions:
- **💥 RAMMING** (Dark Orange) - Actively ramming another car
- **⚡ NITRO ATTACK** (Orange) - Using nitro to overtake
- **⚡ FORCED NITRO ATTACK** (Orange) - Manager-commanded nitro overtake
- **🏎 OVERTAKING** (Orange) - Attempting to pass
- **🎯 FORCED OVERTAKE** (Orange) - Manager-commanded overtake

### Defensive Actions:
- **🛡 DEFENDING** (Blue) - Blocking car behind
- **🛡 NITRO DEFENSE** (Cyan) - Using nitro to defend position

### Special States:
- **⚡ BOOSTING** (Yellow) - Using nitro on straight
- **⚔ RIVALRY BATTLE** (Magenta) - Fighting with rival
- **🔴 BRAKING** (Red) - Slowing for corner

---

## Line 2: Next Intention (What AI Will Do NEXT)

Predicts the AI's next move:

### Corner Predictions:
- `→ Will brake for sharp corner` - Sharp corner ahead (>30% curvature)
- `→ Will slow for corner` - Moderate corner ahead (15-30% curvature)

### Overtaking Plans:
- `→ Completing overtake` - Currently overtaking, will finish
- `→ Planning overtake` - Car ahead, considering pass
- `→ Looking for nitro opportunity` - Has nitro, waiting for chance

### Defensive Plans:
- `→ Holding position` - Defending from car behind
- `→ Conserving nitro` - Low nitro, saving it

### Default:
- `→ Following racing line` - Normal racing, no special plans

---

## Line 3: Decision Reason (WHY AI is Doing This)

Explains the AI's decision-making:

### Ramming Reasons:
- `Ramming (Aggro:81%)` - Shows aggression level that triggered ramming
- `Ramming opportunity` - Generic ramming reason

### Nitro Reasons:
- `Nitro: Conservative strategy` - Using conservative nitro approach
- `Nitro: Aggressive strategy` - Using aggressive nitro approach
- `Nitro: Defensive strategy` - Using defensive nitro approach
- `Nitro: Opportunistic strategy` - Waiting for perfect moment
- `Nitro: Desperate strategy` - Far behind, using nitro frequently

### Overtaking Reasons:
- `Stuck behind slower car` - Overtaking because car ahead is slow

### Corner Reasons:
- `Corner detected (45% sharp)` - Shows corner sharpness percentage

### Personality Reasons:
- `Aggressive driving style` - High aggression personality
- `Veteran precision` - High skill personality
- `Consistent pace` - High consistency personality
- `Taking risks` - High risk-taking personality
- `Conservative approach` - Low aggression/risk personality

---

## Line 4: Internal State (AI's Stats)

Shows key internal values:

Format: `P{position} | {personality} | Nitro:{amount}% | RB:{multiplier}x`

### Examples:
- `P1 | Veteran | Nitro:85% | RB:0.92x`
  - Position 1st
  - Veteran personality
  - 85% nitro remaining
  - 0.92x rubber banding (8% slower - leader penalty)

- `P5 | Rookie | Nitro:23% | RB:1.15x`
  - Position 5th
  - Rookie personality
  - 23% nitro remaining
  - 1.15x rubber banding (15% faster - trailing boost)

- `P3 | Aggressive | Nitro:0% | RB:1.00x`
  - Position 3rd
  - Aggressive personality
  - No nitro left
  - 1.00x rubber banding (no boost/penalty)

### Rubber Banding Values:
- **< 1.0** = Leader penalty (slowed down)
- **= 1.0** = No adjustment
- **> 1.0** = Trailing boost (sped up)

---

## Line 5: Speed

Current speed in km/h, updated in real-time.

---

## Color Coding

### Action Colors:
- **Green** = Normal racing
- **Yellow** = Nitro boost
- **Orange** = Overtaking/attacking
- **Cyan** = Nitro defense
- **Blue** = Defending position
- **Red** = Braking
- **Dark Orange** = Ramming
- **Magenta** = Rivalry battle
- **Gray** = Waiting/inactive
- **White** = Slow/cruising

---

## What to Look For During Testing

### 1. Personality Consistency
Watch multiple AI with same personality:
- **Aggressive**: Should show "Aggressive driving style", ram frequently, use nitro often
- **Conservative**: Should show "Conservative approach", rarely ram, save nitro
- **Veteran**: Should show "Veteran precision", maintain positions well
- **Rookie**: Should show lower positions, get more rubber banding help

### 2. Decision Making
Check if decisions make sense:
- Braking before corners? ✅
- Using nitro on straights? ✅
- Overtaking slower cars? ✅
- Defending when pressured? ✅

### 3. Forced vs Natural Overtaking
- **🎯 FORCED OVERTAKE** = Race manager commanded it
- **🏎 OVERTAKING** = AI decided naturally
- Forced should have priority over natural

### 4. Rubber Banding
- Leader (P1) should have RB < 1.0 (penalty)
- Trailing cars should have RB > 1.0 (boost)
- High-skill AI should have less extreme RB values

### 5. Nitro Strategy
Watch the "Decision Reason" line:
- Conservative AI: Should show "Conservative strategy"
- Aggressive AI: Should show "Aggressive strategy"
- Strategy should match personality type

### 6. Ramming Behavior
- Aggressive AI: Should ram often (70-80% chance)
- Conservative AI: Should rarely ram (5-20% chance)
- Check "Ramming (Aggro:XX%)" to see aggression level

---

## Troubleshooting

### Not Seeing Visualization?
1. Check `showVisualization` is enabled in AIBehaviorVisualizer component
2. Make sure you're in Scene view (not Game view)
3. Verify AIBehaviorVisualizer component is attached to AI cars

### Text Too Small/Large?
Adjust in AIBehaviorVisualizer:
- `heightAboveCar` - Overall height above car
- `lineSpacing` - Space between lines

### Text Flickering?
- Increase `stateDisplayDuration` (default 0.5s)
- Higher values = less frequent updates = less flickering

### Wrong Information?
- Check AIVehicleController is working correctly
- Verify AIPersonalityManager is assigned
- Ensure AIRaceManager is in scene

---

## Example Race Scenarios

### Scenario 1: Aggressive AI Attacking
```
⚡ NITRO ATTACK
→ Completing overtake
Nitro: Aggressive strategy
P4 | Aggressive | Nitro:45% | RB:1.08x
156 km/h
```
**What's happening**: Aggressive AI in 4th place using nitro to overtake, has 45% nitro left, getting 8% speed boost from rubber banding.

### Scenario 2: Veteran AI Leading
```
🏁 RACING
→ Will brake for sharp corner
Veteran precision
P1 | Veteran | Nitro:92% | RB:0.95x
138 km/h
```
**What's happening**: Veteran AI leading the race, about to brake for corner, has lots of nitro saved, getting 5% speed penalty as leader.

### Scenario 3: Rookie AI Struggling
```
🔴 BRAKING
→ Following racing line
Corner detected (38% sharp)
P7 | Rookie | Nitro:12% | RB:1.18x
89 km/h
```
**What's happening**: Rookie AI in 7th place braking for corner, low on nitro, getting 18% speed boost to help catch up.

### Scenario 4: Hothead AI Ramming
```
💥 RAMMING
→ Holding position
Ramming (Aggro:81%)
P2 | Hothead | Nitro:67% | RB:1.02x
145 km/h
```
**What's happening**: Hothead AI in 2nd place ramming (81% aggression chance), defending position, slight speed boost.

---

## Quick Reference

| Symbol | Meaning |
|--------|---------|
| 🏁 | Normal racing |
| ⚡ | Nitro boost |
| 💥 | Ramming |
| 🏎 | Overtaking |
| 🎯 | Forced overtake |
| 🛡 | Defending |
| 🔴 | Braking |
| ⚔ | Rivalry |
| ⏸ | Waiting |
| 🐌 | Slow |
| → | Next action |

---

## Tips for Testing

1. **Watch one AI for a full lap** - See how decisions change
2. **Compare same personalities** - Should behave similarly
3. **Watch leader vs trailer** - Check rubber banding differences
4. **Look for forced overtakes** - Verify manager commands work
5. **Check corner braking** - Should brake before sharp corners
6. **Monitor nitro usage** - Should match personality strategy

The visualization updates every 0.5 seconds, so you'll see real-time decision-making as the AI races!
