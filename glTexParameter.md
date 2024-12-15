##### glTexParameter

`glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_COMPARE_FUNC, GL_EQUAL);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);`

`GL_TEXTURE_COMPARE_MODE`: Determines the comparison mode. 

`GL_COMPARE_REF_TO_TEXTURE` enables depth comparison for shadow mapping.

`GL_TEXTURE_COMPARE_FUNC`: Specifies the comparison function, in this case, GL_EQUAL.

##### textureProj

`float textureProj(sampler2DShadow sampler, vec3 P);`

The sampler is `sampler2DShadow`.
The second parameter is a `vec3 (shadowMapCoord).`

performs depth comparison using `sampler2DShadow` and returns the resulting visibility.