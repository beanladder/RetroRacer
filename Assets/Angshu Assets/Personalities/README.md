# AI Personality System

This folder contains AI personality ScriptableObjects that define how AI racers behave during races.

## Quick Start

### Creating Personality Presets

1. Go to **Tools > AI > Create Personality Presets** in the Unity menu
2. This will create 8 preset personalities (one for each type)
3. You can then customize these presets or use them as-is

### Creating Custom Personalities

1. Go to **Tools > AI > Create Custom Personality** in the Unity menu
2. Or right-click in Project window: **Create > AI > Personality**
3. Customize the values in the Inspector

## Assigning Personalities to AI Racers

### Method 1: Via Race Manager (Recommended)

1. Select your **AIRaceManager** in the scene
2. In the Inspector, find **AI Personality Configuration**
3. Add personality ScriptableObjects to the **Ai Personalities** list
4. Choose whether to assign them randomly or in order
5. When the race starts, personalities will be automatically assigned to AI cars

### Method 2: Direct Assignment

1. Select an AI car prefab or instance
2. Add or find the **AIPersonalityManager** component
3. Assign a personality ScriptableObject to the **Personality Data** field
4. Uncheck **Randomize Personality** to use the assigned personality

## Personality Types

### Aggressive
- High aggression, low patience
- Takes risks and fights for position
- Uses nitro aggressively

### Conservative
- Low aggression, high patience
- Avoids risks, drives consistently
- Conserves nitro for strategic moments

### Opportunist
- Medium aggression
- Waits for the right moment to strike
- Strong comeback drive when behind

### Hothead
- Very aggressive, low pressure resistance
- Makes mistakes under pressure
- Extremely aggressive with nitro and blocking

### Veteran
- High skill and consistency
- Strategic nitro usage
- Excellent pressure resistance

### Rookie
- Low skill, inconsistent
- Conserves nitro (doesn't know when to use it)
- Poor pressure resistance

### Blocker
- Defensive driving style
- Excels at blocking opponents
- Uses nitro defensively

### Speedster
- Focuses on pure speed
- Less tactical, more pace-focused
- Low blocking tendency

## Personality Attributes

Each personality has these customizable attributes:

### Core Traits
- **Aggression**: How aggressive the AI drives
- **Skill**: Driving skill level (precision, consistency)
- **Consistency**: Reduces random variations
- **Risk Taking**: Willingness to take risks
- **Patience**: How long AI waits before overtaking

### Nitro Strategy
- **Nitro Aggression**: Tendency to use nitro for attacks
- **Nitro Defense**: Tendency to use nitro defensively
- **Nitro Conservation**: Tendency to save nitro

### Racing Behavior
- **Blocking Tendency**: How often AI blocks others
- **Overtaking Aggression**: Aggression when overtaking
- **Defensive Driving**: How defensively AI drives under pressure

### Pressure Response
- **Pressure Resistance**: Performance under stress
- **Comeback Drive**: Drive to catch up when behind

## Tips

- Mix different personality types for more interesting races
- Veteran + Rookie combinations create mentor/student rivalries
- Aggressive + Hothead personalities will clash and create intense battles
- Blocker + Speedster combinations create natural conflicts
- Use 3-5 different personalities for variety without overwhelming complexity
