🏎️ Rebuilt the AI opponents for RetroRacer — they now drive on real physics, not scripted rules

🎯 **The Challenge:**
Make AI cars that can handle a brand-new procedurally generated track every run, brake at the right point for their own grip, and actually race each other — not just the clock.

🧠 **How they think:**

🏁 Speed comes from grip, not guesswork
• Corner speed = how much lateral grip the car has ÷ how sharp the curve is
• The car plans backward from every corner to find its real braking point
• Braking zones come from physics, not a hand-tuned "slow down here" hack

🎮 Steering that matches the car underneath it
• AI aims down the track and solves the exact arc to get there
• That steering angle is translated through the car's real turning limits — no cheating the physics
• Braking uses the throttle, not the handbrake — using the handbrake here would literally spin the car out

🚦 Traffic awareness, car to car
• Every car tracks exactly how far ahead/behind and side-to-side every rival is
• Overtakes only trigger when there's real room, and hold for a beat before committing
• AI and player are scored by the exact same lap + distance formula, so positions are always fair

📈 Cars that learn mid-race
• Each AI watches how much it's sliding through corners
• Slide too much → it backs off and brakes earlier next time
• Feels less like a script, more like a driver finding their limit

👀 Watch the AI think, live
• Color-coded lines now show up right in the Game view: red = about to pass, magenta = passing now, blue = just watching
• Added a free camera to fly around and spectate any car on the grid mid-race

🐛 **Best bug of the project:**
Found a field the AI used to check its own speed... that nothing in the code ever actually updated. Every AI car had secretly believed it was standing still this whole time. 😅

✅ **Result:** A full grid of AI drivers that brake like they mean it, hold a line through corners, and genuinely race for position — on tracks they've never seen before.

#GameDev #GameAI #Unity3D #ProceduralGeneration #RacingGames
