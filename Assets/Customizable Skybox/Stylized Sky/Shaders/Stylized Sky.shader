Shader "Stylized/Sky"
{
    Properties
    {
        [Header(Sun Disc)]
        _SunDiscColor ("Color", Color) = (1, 1, 1, 1)
        _SunDiscMultiplier ("Multiplier", float) = 25
        _SunDiscExponent ("Exponent", float) = 125000
        
        [Header(Sun Halo)]
        _SunHaloColor ("Color", Color) = (0.8970588, 0.7760561, 0.6661981, 1)
        _SunHaloExponent ("Exponent", float) = 125
        _SunHaloContribution ("Contribution", Range(0, 1)) = 0.75
        
        [Header(Horizon Line)]
        _HorizonLineColor ("Color", Color) = (0.9044118, 0.8872592, 0.7913603, 1)
        _HorizonLineExponent ("Exponent", float) = 4
        _HorizonLineContribution ("Contribution", Range(0, 1)) = 0.25
       
        [Header(Sky Gradient)]
        _SkyGradientTop ("Top", Color) = (0.172549, 0.5686274, 0.6941177, 1)
        _SkyGradientBottom ("Bottom", Color) = (0.764706, 0.8156863, 0.8509805)
        _SkyGradientExponent ("Exponent", float) = 2.5
        
        [Header(Stars)]
        [Toggle] _EnableStars ("Enable Stars", Float) = 1
        _StarColor ("Star Color", Color) = (1, 1, 1, 1)
        _StarDensity ("Star Density", Range(1, 100)) = 50
        _StarIntensity ("Star Intensity", Range(0, 2)) = 1
        _StarTwinkleSpeed ("Twinkle Speed", Range(0, 10)) = 3
        _StarSize ("Star Size", Range(0.01, 0.1)) = 0.05
    }
    SubShader
    {
        Tags
        {
            "RenderType" = "Background"
            "Queue" = "Background"
        }
        LOD 100
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
           
            float3 _SunDiscColor;
            float _SunDiscExponent;
            float _SunDiscMultiplier;
            float3 _SunHaloColor;
            float _SunHaloExponent;
            float _SunHaloContribution;
            float3 _HorizonLineColor;
            float _HorizonLineExponent;
            float _HorizonLineContribution;
            float3 _SkyGradientTop;
            float3 _SkyGradientBottom;
            float _SkyGradientExponent;
            
            // Star properties
            float _EnableStars;
            float3 _StarColor;
            float _StarDensity;
            float _StarIntensity;
            float _StarTwinkleSpeed;
            float _StarSize;
            
            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };
            
            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
                float3 worldPosition : TEXCOORD1;
            };
            
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.worldPosition = mul(unity_ObjectToWorld, v.vertex).xyz;
                return o;
            }
            
            // Hash function for pseudo-random number generation
            float hash(float2 p)
            {
                p = frac(p * float2(123.34, 456.21));
                p += dot(p, p + 45.32);
                return frac(p.x * p.y);
            }
            
            // Function to generate stars
            float3 GenerateStars(float3 dir)
            {
                // Only generate stars in upper hemisphere
                if (dir.y < 0.0) return float3(0, 0, 0);
                
                // Convert direction to spherical coordinates
                float2 uv = float2(atan2(dir.z, dir.x), asin(dir.y));
                uv *= _StarDensity;
                
                // Generate grid cells for stars
                float2 gv = frac(uv) - 0.5;
                float2 id = floor(uv);
                
                float3 starColor = float3(0, 0, 0);
                
                // Sample nearby grid cells
                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        float2 offset = float2(x, y);
                        float2 cellId = id + offset;
                        
                        // Random position within the cell
                        float cellHash = hash(cellId);
                        float starBrightness = hash(cellId + 0.1);
                        
                        // Random offset within cell
                        float2 cellGv = gv - offset - float2(hash(cellId + 0.3) - 0.5, hash(cellId + 0.4) - 0.5);
                        
                        // Twinkle effect
                        float twinkle = sin(_Time.y * _StarTwinkleSpeed * cellHash) * 0.5 + 0.5;
                        
                        // Star shape (simple distance function)
                        float star = smoothstep(_StarSize, 0.0, length(cellGv));
                        star *= starBrightness * twinkle;
                        
                        starColor += star * _StarColor * _StarIntensity;
                    }
                }
                
                return starColor;
            }
            
            fixed4 frag (v2f i) : SV_Target
            {
                // Normalize the world position for direction
                float3 viewDir = normalize(i.worldPosition);
                
                // Masks
                float maskHorizon = dot(viewDir, float3(0, 1, 0));
                float maskSunDir = dot(viewDir, _WorldSpaceLightPos0.xyz);
                
                // Sun disc
                float maskSun = pow(saturate(maskSunDir), _SunDiscExponent);
                maskSun = saturate(maskSun * _SunDiscMultiplier);
                
                // Sun halo
                float3 sunHaloColor = _SunHaloColor * _SunHaloContribution;
                float bellCurve = pow(saturate(maskSunDir), _SunHaloExponent * saturate(abs(maskHorizon)));
                float horizonSoften = 1 - pow(1 - saturate(maskHorizon), 50);
                sunHaloColor *= saturate(bellCurve * horizonSoften);
                
                // Horizon line
                float3 horizonLineColor = _HorizonLineColor * saturate(pow(1 - abs(maskHorizon), _HorizonLineExponent));
                horizonLineColor = lerp(0, horizonLineColor, _HorizonLineContribution);
                
                // Sky gradient
                float3 skyGradientColor = lerp(_SkyGradientTop, _SkyGradientBottom, pow(1 - saturate(maskHorizon), _SkyGradientExponent));
                
                // Base sky color
                float3 finalColor = lerp(saturate(sunHaloColor + horizonLineColor + skyGradientColor), _SunDiscColor, maskSun);
                
                // Add stars if enabled
                if (_EnableStars > 0.5)
                {
                    float3 stars = GenerateStars(viewDir);
                    // Only add stars to the darker parts of the sky and avoid adding them to the sun
                    float starMask = (1.0 - maskSun) * (1.0 - saturate(dot(finalColor, float3(0.299, 0.587, 0.114)) * 2.0));
                    finalColor += stars * starMask;
                }
                
                return float4(finalColor, 1);
            }
            ENDCG
        }
    }
}