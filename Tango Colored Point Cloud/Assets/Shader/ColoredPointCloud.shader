Shader "Tango/ColoredPointCloud" {
Properties{
        point_size("Point Size", Float) = 6.0
}
  SubShader {
     Pass {
        GLSLPROGRAM
        
        #ifdef VERTEX
        uniform mat4 depthCameraTUnityWorld;
        uniform float point_size;
        varying vec4 v_color;
        void main()
        {   
           gl_PointSize = point_size;
           gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
           v_color = gl_Color;
        }
        #endif

        #ifdef FRAGMENT
        varying vec4 v_color;
        void main()
        {
           gl_FragColor = v_color;
        }
        #endif

        ENDGLSL
     }
  }
}