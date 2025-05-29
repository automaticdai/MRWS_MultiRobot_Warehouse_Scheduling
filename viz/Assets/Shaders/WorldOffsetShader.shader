Shader "Unlit/Checkerboard"
{
    Properties
    {
        _Density ("Density", Range(1,50)) = 1
    }
    SubShader
    {
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            float _Density;

            v2f vert (float4 pos : POSITION, float2 uv : TEXCOORD0)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(pos);
                float4x4 obj2world = unity_ObjectToWorld;
                float4 transform = mul(obj2world, pos);
                //o.uv = uv * _Density;
                o.uv = transform.xz * _Density;
                return o;
            }
            
            fixed4 frag (v2f i) : SV_Target
            {
                float2 c = i.uv;
                c = floor(c) / 2;
                float checker = frac(c.x + c.y) * 0.5;
                return checker;
            }
            ENDCG
        }
    }
}