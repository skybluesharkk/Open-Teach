// 트래킹된 손 메시 자체를 물들이는 히트맵 셰이더 (Built-in RP, Quest)
// TactileOverlay가 손끝 5개 히트 포인트(_HeatPts: xyz=월드좌표, w=강도)를 매 프레임 설정.
// 손 표면 각 픽셀이 히트 포인트와의 거리로 jet 컬러맵을 입음 — 별도 레이어 없이 손 그 자체가 변색.
Shader "Tactile/HandHeat"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.13, 0.13, 0.16, 1)
        _HeatRadius ("Heat Radius (m)", Float) = 0.016
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            // 배열(SetVectorArray)은 Vulkan/IL2CPP에서 전달이 조용히 실패하는
            // 사례가 있어 개별 유니폼 5개로 전달 (xyz: world pos, w: intensity 0~1)
            float4 _HeatPt0, _HeatPt1, _HeatPt2, _HeatPt3, _HeatPt4;
            float4 _BaseColor;
            float  _HeatRadius;

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float3 wpos  : TEXCOORD0;
                float3 wnorm : TEXCOORD1;
            };

            v2f vert (appdata_base v)
            {
                v2f o;
                o.pos   = UnityObjectToClipPos(v.vertex);
                o.wpos  = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.wnorm = UnityObjectToWorldNormal(v.normal);
                return o;
            }

            // 6단계 jet 램프: 파랑→시안→초록→노랑→주황→빨강 (TactileOverlay.HeatStops와 동일)
            fixed3 jetRamp (float k)
            {
                const float3 c0 = float3(0.09, 0.22, 0.66);
                const float3 c1 = float3(0.00, 0.62, 0.95);
                const float3 c2 = float3(0.13, 0.79, 0.35);
                const float3 c3 = float3(1.00, 0.86, 0.10);
                const float3 c4 = float3(0.98, 0.45, 0.02);
                const float3 c5 = float3(0.84, 0.04, 0.07);
                k = saturate(k) * 5.0;
                float3 col = lerp(c0, c1, saturate(k));
                col = lerp(col, c2, saturate(k - 1.0));
                col = lerp(col, c3, saturate(k - 2.0));
                col = lerp(col, c4, saturate(k - 3.0));
                col = lerp(col, c5, saturate(k - 4.0));
                return col;
            }

            float heatFrom (float3 wpos, float4 pt, float r2)
            {
                float3 d = wpos - pt.xyz;
                return pt.w * exp(-dot(d, d) / r2);
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float r2 = _HeatRadius * _HeatRadius;
                float heat = heatFrom(i.wpos, _HeatPt0, r2)
                           + heatFrom(i.wpos, _HeatPt1, r2)
                           + heatFrom(i.wpos, _HeatPt2, r2)
                           + heatFrom(i.wpos, _HeatPt3, r2)
                           + heatFrom(i.wpos, _HeatPt4, r2);
                heat = saturate(heat);

                // 은은한 램버트 근사 조명 (기본 손 색이 완전 평면으로 보이지 않게)
                float ndl = saturate(dot(normalize(i.wnorm), normalize(_WorldSpaceLightPos0.xyz))) * 0.45 + 0.55;
                float3 baseC = _BaseColor.rgb * ndl;
                // 임계 상향: 미약한 원거리 히트가 손바닥 전체를 물들이지 않게
                float3 col = lerp(baseC, jetRamp(heat), smoothstep(0.06, 0.35, heat));
                return fixed4(col, 1.0);
            }
            ENDCG
        }
    }
}
