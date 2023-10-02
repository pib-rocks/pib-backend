package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;
import io.swagger.v3.oas.annotations.media.Schema;
import java.math.BigDecimal;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetAllPersonalitiesVoiceAssistantPersonalities
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetAllPersonalitiesVoiceAssistantPersonalities   {
  @JsonProperty("personality_id")
  private String personalityId = null;

  @JsonProperty("name")
  private String name = null;

  @JsonProperty("description")
  private String description = null;

  /**
   * Gets or Sets gender
   */
  public enum GenderEnum {
    MALE("Male"),
    
    FEMALE("Female");

    private String value;

    GenderEnum(String value) {
      this.value = value;
    }

    @Override
    @JsonValue
    public String toString() {
      return String.valueOf(value);
    }

    @JsonCreator
    public static GenderEnum fromValue(String text) {
      for (GenderEnum b : GenderEnum.values()) {
        if (String.valueOf(b.value).equals(text)) {
          return b;
        }
      }
      return null;
    }
  }
  @JsonProperty("gender")
  private GenderEnum gender = null;

  @JsonProperty("pauseThreshold")
  private BigDecimal pauseThreshold = null;

  public GetAllPersonalitiesVoiceAssistantPersonalities personalityId(String personalityId) {
    this.personalityId = personalityId;
    return this;
  }

  /**
   * Get personalityId
   * @return personalityId
   **/
  @Schema(example = "UUID", description = "")
  
    public String getPersonalityId() {
    return personalityId;
  }

  public void setPersonalityId(String personalityId) {
    this.personalityId = personalityId;
  }

  public GetAllPersonalitiesVoiceAssistantPersonalities name(String name) {
    this.name = name;
    return this;
  }

  /**
   * Get name
   * @return name
   **/
  @Schema(example = "Georg", description = "")
  
    public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public GetAllPersonalitiesVoiceAssistantPersonalities description(String description) {
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

  public GetAllPersonalitiesVoiceAssistantPersonalities gender(GenderEnum gender) {
    this.gender = gender;
    return this;
  }

  /**
   * Get gender
   * @return gender
   **/
  @Schema(example = "Male", description = "")
  
    public GenderEnum getGender() {
    return gender;
  }

  public void setGender(GenderEnum gender) {
    this.gender = gender;
  }

  public GetAllPersonalitiesVoiceAssistantPersonalities pauseThreshold(BigDecimal pauseThreshold) {
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
    GetAllPersonalitiesVoiceAssistantPersonalities getAllPersonalitiesVoiceAssistantPersonalities = (GetAllPersonalitiesVoiceAssistantPersonalities) o;
    return Objects.equals(this.personalityId, getAllPersonalitiesVoiceAssistantPersonalities.personalityId) &&
        Objects.equals(this.name, getAllPersonalitiesVoiceAssistantPersonalities.name) &&
        Objects.equals(this.description, getAllPersonalitiesVoiceAssistantPersonalities.description) &&
        Objects.equals(this.gender, getAllPersonalitiesVoiceAssistantPersonalities.gender) &&
        Objects.equals(this.pauseThreshold, getAllPersonalitiesVoiceAssistantPersonalities.pauseThreshold);
  }

  @Override
  public int hashCode() {
    return Objects.hash(personalityId, name, description, gender, pauseThreshold);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetAllPersonalitiesVoiceAssistantPersonalities {\n");
    
    sb.append("    personalityId: ").append(toIndentedString(personalityId)).append("\n");
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
