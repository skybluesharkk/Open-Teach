// 택타일 히트맵 셸용 정점 색상 셰이더 (Built-in RP, Quest 호환)
// TactileOverlay.cs가 손끝 셸 메시의 정점 색을 매 프레임 갱신 → 부드러운 heat blob
Shader "Tactile/HeatVertex"
{
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" "IgnoreProjector"="True" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            struct appdata { float4 vertex : POSITION; fixed4 color : COLOR; };
            struct v2f { float4 pos : SV_POSITION; fixed4 col : COLOR; };
            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.col = v.color;
                return o;
            }
            fixed4 frag (v2f i) : SV_Target { return i.col; }
            ENDCG
        }
    }
}
