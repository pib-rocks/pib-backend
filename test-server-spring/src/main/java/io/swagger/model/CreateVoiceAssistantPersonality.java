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
 * CreateVoiceAssistantPersonality
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-09-27T09:57:56.878396287Z[GMT]")


public class CreateVoiceAssistantPersonality   {
  @JsonProperty("name")
  private String name = null;

  @JsonProperty("description")
  private String description = null;

  @JsonProperty("gender")
  private String gender = null;

  @JsonProperty("pauseThreshold")
  private BigDecimal pauseThreshold = null;

  public CreateVoiceAssistantPersonality name(String name) {
    this.name = name;
    return this;
  }

  /**
   * Get name
   * @return name
   **/
  @Schema(example = "Name of the voice assistant personality", required = true, description = "")
      @NotNull

    public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public CreateVoiceAssistantPersonality description(String description) {
    this.description = description;
    return this;
  }

  /**
   * Get description
   * @return description
   **/
  @Schema(example = "Personality Description", required = true, description = "")
      @NotNull

    public String getDescription() {
    return description;
  }

  public void setDescription(String description) {
    this.description = description;
  }

  public CreateVoiceAssistantPersonality gender(String gender) {
    this.gender = gender;
    return this;
  }

  /**
   * Get gender
   * @return gender
   **/
  @Schema(example = "Male/Female", required = true, description = "")
      @NotNull

    public String getGender() {
    return gender;
  }

  public void setGender(String gender) {
    this.gender = gender;
  }

  public CreateVoiceAssistantPersonality pauseThreshold(BigDecimal pauseThreshold) {
    this.pauseThreshold = pauseThreshold;
    return this;
  }

  /**
   * Get pauseThreshold
   * @return pauseThreshold
   **/
  @Schema(example = "0.8", required = true, description = "")
      @NotNull

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
    CreateVoiceAssistantPersonality createVoiceAssistantPersonality = (CreateVoiceAssistantPersonality) o;
    return Objects.equals(this.name, createVoiceAssistantPersonality.name) &&
        Objects.equals(this.description, createVoiceAssistantPersonality.description) &&
        Objects.equals(this.gender, createVoiceAssistantPersonality.gender) &&
        Objects.equals(this.pauseThreshold, createVoiceAssistantPersonality.pauseThreshold);
  }

  @Override
  public int hashCode() {
    return Objects.hash(name, description, gender, pauseThreshold);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class CreateVoiceAssistantPersonality {\n");
    
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
