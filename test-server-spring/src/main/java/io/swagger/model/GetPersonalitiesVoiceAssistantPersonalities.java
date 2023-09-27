package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import java.math.BigDecimal;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetPersonalitiesVoiceAssistantPersonalities
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-09-27T09:57:56.878396287Z[GMT]")


public class GetPersonalitiesVoiceAssistantPersonalities   {
  @JsonProperty("id")
  private String id = null;

  @JsonProperty("name")
  private String name = null;

  @JsonProperty("description")
  private String description = null;

  @JsonProperty("gender")
  private String gender = null;

  @JsonProperty("pauseThreshold")
  private BigDecimal pauseThreshold = null;

  public GetPersonalitiesVoiceAssistantPersonalities id(String id) {
    this.id = id;
    return this;
  }

  /**
   * Get id
   * @return id
   **/
  @Schema(example = "UUID", description = "")
  
    public String getId() {
    return id;
  }

  public void setId(String id) {
    this.id = id;
  }

  public GetPersonalitiesVoiceAssistantPersonalities name(String name) {
    this.name = name;
    return this;
  }

  /**
   * Get name
   * @return name
   **/
  @Schema(example = "Voice Assistant Personality Name", description = "")
  
    public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public GetPersonalitiesVoiceAssistantPersonalities description(String description) {
    this.description = description;
    return this;
  }

  /**
   * Get description
   * @return description
   **/
  @Schema(example = "Personality Description", description = "")
  
    public String getDescription() {
    return description;
  }

  public void setDescription(String description) {
    this.description = description;
  }

  public GetPersonalitiesVoiceAssistantPersonalities gender(String gender) {
    this.gender = gender;
    return this;
  }

  /**
   * Get gender
   * @return gender
   **/
  @Schema(example = "Male/Female", description = "")
  
    public String getGender() {
    return gender;
  }

  public void setGender(String gender) {
    this.gender = gender;
  }

  public GetPersonalitiesVoiceAssistantPersonalities pauseThreshold(BigDecimal pauseThreshold) {
    this.pauseThreshold = pauseThreshold;
    return this;
  }

  /**
   * Get pauseThreshold
   * @return pauseThreshold
   **/
  @Schema(example = "0.8", description = "")
  
    @Valid
    public BigDecimal getPauseThreshold() {
    return pauseThreshold;
  }

  public void setPauseThreshold(BigDecimal pauseThreshold) {
    this.pauseThreshold = pauseThreshold;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    GetPersonalitiesVoiceAssistantPersonalities getPersonalitiesVoiceAssistantPersonalities = (GetPersonalitiesVoiceAssistantPersonalities) o;
    return Objects.equals(this.id, getPersonalitiesVoiceAssistantPersonalities.id) &&
        Objects.equals(this.name, getPersonalitiesVoiceAssistantPersonalities.name) &&
        Objects.equals(this.description, getPersonalitiesVoiceAssistantPersonalities.description) &&
        Objects.equals(this.gender, getPersonalitiesVoiceAssistantPersonalities.gender) &&
        Objects.equals(this.pauseThreshold, getPersonalitiesVoiceAssistantPersonalities.pauseThreshold);
  }

  @Override
  public int hashCode() {
    return Objects.hash(id, name, description, gender, pauseThreshold);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetPersonalitiesVoiceAssistantPersonalities {\n");
    
    sb.append("    id: ").append(toIndentedString(id)).append("\n");
    sb.append("    name: ").append(toIndentedString(name)).append("\n");
    sb.append("    description: ").append(toIndentedString(description)).append("\n");
    sb.append("    gender: ").append(toIndentedString(gender)).append("\n");
    sb.append("    pauseThreshold: ").append(toIndentedString(pauseThreshold)).append("\n");
    sb.append("}");
    return sb.toString();
  }

  /**
   * Convert the given object to string with each line indented by 4 spaces
   * (except the first line).
   */
  private String toIndentedString(java.lang.Object o) {
    if (o == null) {
      return "null";
    }
    return o.toString().replace("\n", "\n    ");
  }
}
